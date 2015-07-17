#include "rtt_lwr_cart_imp_ctrl/rtt_lwr_cart_imp_ctrl.hpp"
#include "eigen_conversions/eigen_msg.h"

#include <unistd.h>

namespace lwr{
using namespace RTT;
using namespace KDL;

RttLwrCartImpCtrl::RttLwrCartImpCtrl(const std::string& name):
t_traj_curr(0),
kp_lin(1000.0),
kp_ang(650.0),
kd_lin(35.0),
kd_ang(5.0),
ready_to_start_(false),
traj_computed(false),
debug_mode_(false),
use_jdot_qdot_(true),
use_f_ext_(true),
use_coriolis_(true),
dx_ang_(10000.0),
use_xdd_des_(true),
use_mass_sqrt_(false),
elapsed(0),
jacobian_solver_type_(WDL_SOLVER),
RTTLWRAbstract(name)
{
    this->ports()->addPort("PathROS",port_path_ros).doc("");
    this->ports()->addPort("X_curr",port_X_curr).doc("");
    this->ports()->addPort("X_tmp",port_X_tmp).doc("");
    this->ports()->addPort("X_des",port_X_des).doc("");
    this->addOperation("publishTrajectory",&RttLwrCartImpCtrl::publishTrajectory,this,RTT::OwnThread);
    this->addOperation("computeTrajectory",&RttLwrCartImpCtrl::computeTrajectory,this,RTT::OwnThread);
    this->addOperation("updateCartesianGains",&RttLwrCartImpCtrl::updateCartesianGains,this,RTT::OwnThread);
    this->addAttribute("kp_lin",kp_lin);
    this->addAttribute("kp_ang",kp_ang);
    this->addAttribute("kd_lin",kd_lin);
    this->addAttribute("kd_ang",kd_ang);
    
    this->addAttribute("dx_ang",dx_ang_);
    this->addAttribute("debug_mode",debug_mode_);
    this->addAttribute("use_jdot_qdot",use_jdot_qdot_);
    this->addAttribute("use_f_ext_",use_f_ext_);
    this->addAttribute("use_coriolis_",use_coriolis_);
    this->addAttribute("ReadyToStart",ready_to_start_);
    this->addAttribute("jacobian_solver_type",jacobian_solver_type_);
    this->addAttribute("elapsed",elapsed);
    this->addAttribute("use_mass_sqrt",use_mass_sqrt_);
}

bool RttLwrCartImpCtrl::configureHook()
{
    bool configure = RTTLWRAbstract::configureHook();
    
    jdot_solver.reset(new ChainJntToJacDotSolver(kdl_chain));
    pinv_solver.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain));
    wdls_solver.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain));
    
    int cnt = 10;
    while(!updateState() && !cnt--)
    {
        log(Warning) << "Waiting for lwr_fri to publish data " << endlog();
        usleep(5e5);
    }
    
    setJointTorqueControlMode();
    
    kp_cart << 550,550,550,50,50,50;
    
    kd_cart << 5,5,5,0.5,0.5,0.5;
    
    Xd_cmd.setZero();
    X_err.setZero();
    Xd_err.setZero();
    
    port_X_curr.createStream(rtt_roscomm::topic("~"+getName()+"/pos_curr"));
    port_X_tmp.createStream(rtt_roscomm::topic("~"+getName()+"/pos_tmp"));
    port_X_des.createStream(rtt_roscomm::topic("~"+getName()+"/pos_des"));
    port_path_ros.createStream(rtt_roscomm::topic("~"+getName()+"/cart_traj_des"));
    
    
    fk_vel_solver->JntToCart(jnt_pos_vel_kdl,frame_vel_des_kdl);
    frame_des_kdl =  frame_vel_des_kdl.GetFrame();
    
    
    qdd_des.resize(kdl_chain.getNrOfJoints());
    mass_kdl.resize(kdl_chain.getNrOfJoints());
    qdd_des_kdl.resize(kdl_chain.getNrOfJoints());
    coriolis.resize(kdl_chain.getNrOfJoints());
    jnt_pos_eigen.resize(kdl_chain.getNrOfJoints());
    jdot.resize(kdl_chain.getNrOfSegments());
    // Get initial pose
    getCartesianPosition(cart_pos);
    
    computeTrajectory(0.01,0.05);   
     
    mass_inv.resize(kdl_chain.getNrOfJoints(),kdl_chain.getNrOfJoints());
    
    updateCartesianGains(kp_lin,kp_ang,kd_lin,kd_ang);
    return configure && cnt;
}
bool RttLwrCartImpCtrl::computeTrajectory(const double radius, const double eqradius)
{
    try
    {
        updateState();
        fk_vel_solver->JntToCart(jnt_pos_vel_kdl,frame_vel_des_kdl,kdl_chain.getNrOfSegments());
        frame_des_kdl =  frame_vel_des_kdl.GetFrame();
       
        path = new Path_RoundedComposite(radius,eqradius,new RotationalInterpolation_SingleAxis());
        path->Add(frame_des_kdl);
        path->Add(Frame(Rotation::RPY(88.*deg2rad,        -4.*deg2rad,     -36.*deg2rad),   Vector(-.372,-.527,.505)));
        path->Add(Frame(Rotation::RPY(91.0*deg2rad,         -3.*deg2rad,   -28.*deg2rad), Vector(-.198,-.657,.695)));
        path->Add(Frame(Rotation::RPY(89.0*deg2rad,       -17.*deg2rad,   -32.*deg2rad), Vector(-.219,-.725,.404)));
        path->Add(frame_des_kdl);
        // always call Finish() at the end, otherwise the last segment will not be added.
        path->Finish();
        
        velpref = new VelocityProfile_Trap(0.1,0.1);
        velpref->SetProfile(0,path->PathLength());  
        traject = new Trajectory_Segment(path, velpref);
        
        ctraject = new Trajectory_Composite();
        ctraject->Add(traject);
        ctraject->Add(new Trajectory_Stationary(1.0,frame_des_kdl));
        
    } catch(KDL::Error& error) {
            std::cout <<"I encountered this error : " << error.Description() << std::endl;
            std::cout << "with the following type " << error.GetType() << std::endl;
            return false;
    }
    
    log(Info) << "Trajectory computed ! " << endlog();
    publishTrajectory();
    traj_computed = true;
    return true;
}

void RttLwrCartImpCtrl::publishTrajectory()
{
    double dt=static_cast<double>(this->getPeriod());
    nav_msgs::Path path_ros;
    path_ros.header.frame_id = root_link;
    log(Debug) << "Creating Path" << endlog();
    for (double t=0.0; t <= traject->Duration(); t+= dt) {
            Frame current_pose;
            Twist current_vel,current_acc;
            
            current_pose = traject->Pos(t);
            current_vel = traject->Vel(t);
            current_acc = traject->Acc(t);
                        
            geometry_msgs::Pose pose;
            geometry_msgs::PoseStamped pose_st;
            tf::poseKDLToMsg(current_pose,pose);
            pose_st.header.frame_id = path_ros.header.frame_id;
            pose_st.pose = pose;
            path_ros.poses.push_back(pose_st);

    }
    log(Debug) << "Sending Path" << endlog();
    port_path_ros.write(path_ros);
    log(Debug) << "Publishing done" << endlog();
}

void RttLwrCartImpCtrl::updateCartesianGains(const double kp_lin_, const double kp_ang_, const double kd_lin_, const double kd_ang_)
{
    cart_imp_cmd.stiffness.linear.x = kp_lin_;
    cart_imp_cmd.stiffness.linear.y = kp_lin_;
    cart_imp_cmd.stiffness.linear.z = kp_lin_;
    
    cart_imp_cmd.stiffness.angular.x = kp_ang_;
    cart_imp_cmd.stiffness.angular.y = kp_ang_;
    cart_imp_cmd.stiffness.angular.z = kp_ang_;
    
    cart_imp_cmd.damping.linear.x = kd_lin_; 
    cart_imp_cmd.damping.linear.y = kd_lin_;
    cart_imp_cmd.damping.linear.z = kd_lin_;
    
    cart_imp_cmd.damping.angular.x = kd_ang_; 
    cart_imp_cmd.damping.angular.y = kd_ang_;
    cart_imp_cmd.damping.angular.z = kd_ang_; 
    
    kp_lin=(kp_lin_);
    
    kp_ang=(kp_ang_);

    
    kd_lin=(kd_lin_);

    

    kd_ang=(kd_ang_);    
    
    
    port_CartesianImpedanceCommand.write(cart_imp_cmd);
}

void RttLwrCartImpCtrl::updateHook()
{
    ros::Time t_start = rtt_rosclock::host_now();
    
    if(!debug_mode_)
        if(!isCommandMode() || !isReadyToStart())
            return;
      
    updateState();

    Frame X_des,X_mes;
    Twist Xdd_des,Xd_mes,Xd_des;
    
    X_des = traject->Pos(t_traj_curr);
    
    // Fk -> X
    jnt_to_jac_solver->JntToJac(jnt_pos_kdl,J,kdl_chain.getNrOfSegments());
    fk_vel_solver->JntToCart(jnt_pos_vel_kdl,tool_in_base_framevel,kdl_chain.getNrOfSegments());
    X_mes =  tool_in_base_framevel.GetFrame();
    Xd_mes = tool_in_base_framevel.GetTwist();
    
    // Flex Models
    jnt_pos_eigen = jnt_pos_kdl.data;
    flex_model_12.compute(jnt_pos_eigen,0.0);
    corr_cart = flex_model_12.corr_trans_rot;
    KDL::Frame X_corr(Rotation::Quaternion(corr_cart[3],corr_cart[4],corr_cart[5],corr_cart[6]),
        Vector(corr_cart[0],corr_cart[1],corr_cart[2]));
    // Xdd Des

    //Twist d_curr = diff(X_corr,X_mes);
    
    //Frame X_curr = KDL::addDelta(X_mes,d_curr,dx_ang_*static_cast<double>(getPeriod()));
    
    Frame X_curr = X_mes;
    
    Twist d_err = diff(X_curr,X_des);    

    double ang_ = d_err.rot.Norm();
    
    if(ang_ >= dx_ang_)
    {
        d_err.rot.Normalize();
        d_err.rot = d_err.rot*dx_ang_;
    }
    
    Rotation dr = Rotation::Rot(d_err.rot,d_err.rot.Norm());
    Frame X_tmp(X_curr);
    X_tmp.M = dr * X_tmp.M;
    
    if(1)
    {
        // Ros pub
       X_curr_msg.header.frame_id = 
            X_tmp_msg.header.frame_id = 
            X_des_msg.header.frame_id = root_link;
            
       X_curr_msg.header.stamp = 
            X_tmp_msg.header.stamp = 
            X_des_msg.header.stamp = rtt_rosclock::host_now();
            
       tf::poseKDLToMsg(X_curr,X_curr_msg.pose);
       tf::poseKDLToMsg(X_tmp,X_tmp_msg.pose);
       tf::poseKDLToMsg(X_des,X_des_msg.pose);
       
       port_X_curr.write(X_curr_msg);
       port_X_tmp.write(X_tmp_msg);
       port_X_des.write(X_des_msg);
    }
    
    // Mass Matrix
    if(0)
        id_dyn_solver->JntToMass(jnt_pos_kdl,mass_kdl);
    else{
        getMassMatrix(mass);
        mass_kdl.data = mass;
    }
    
    
    Xd_des = traject->Vel(t_traj_curr);
    Twist dd_err = diff(Xd_mes,Xd_des);
    
    
    
    Xdd_des.vel = kp_lin*d_err.vel + kd_lin*dd_err.vel;
    Xdd_des.rot = kp_ang*d_err.rot + kd_ang*(dd_err.rot);
    
    // Jdot*qdot
    jdot_solver->JntToJacDot(jnt_pos_vel_kdl,jdot_qdot,kdl_chain.getNrOfSegments());
    
    if(use_jdot_qdot_)
        Xdd_des -= jdot_qdot;
    
    if(use_xdd_des_)
        Xdd_des += traject->Acc(t_traj_curr);
    

    
    // qdd = pinv(J)*(Xdd - Jdot*qdot)
    int ret=0;
    switch(jacobian_solver_type_)
    {
        case PINV_SOLVER:
            ret = pinv_solver->CartToJnt(jnt_pos_kdl,Xdd_des,qdd_des_kdl);
            break;
        case WDL_SOLVER:
            mass_inv = mass.inverse();
            if(use_mass_sqrt_)
                wdls_solver->setWeightJS(mass_inv.sqrt());
            else
                wdls_solver->setWeightJS(mass_inv);
            ret = wdls_solver->CartToJnt(jnt_pos_kdl,Xdd_des,qdd_des_kdl);
            break;
        default:
            log(RTT::Error) << "Invalid mode" << endlog();
            this->error();
            
    }
    
    
    id_dyn_solver->JntToCoriolis(jnt_pos_kdl,jnt_vel_kdl,coriolis_kdl);
    
    F_ext.setZero();
    // Du capteur effort, à remettre dans le repère Monde;
    
    jnt_trq_cmd = mass*qdd_des_kdl.data;
    if(use_f_ext_)
        jnt_trq_cmd += J.data.transpose() * F_ext;
    if(use_coriolis_)
        jnt_trq_cmd += coriolis_kdl.data;
    
    log(Debug) << "trqcmd " << jnt_trq_cmd.transpose() << endlog();
    
    if(!debug_mode_)
        sendJointTorque(jnt_trq_cmd);
    
    // Incremente traj
    if( t_traj_curr <= traject->Duration())
        t_traj_curr += static_cast<double>(this->getPeriod());
    else
        t_traj_curr = 0.0;
    
    ros::Duration t_elapsed = rtt_rosclock::host_now() - t_start;
    elapsed = t_elapsed.toSec();
}

}
