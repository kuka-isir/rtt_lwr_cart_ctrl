    #include "rtt_lwr_cart_ctrl/rtt_lwr_cart_ctrl.hpp"
#include "eigen_conversions/eigen_msg.h"

#include <unistd.h>

namespace lwr{
using namespace RTT;
using namespace KDL;

RttLwrCartCtrl::RttLwrCartCtrl(const std::string& name):
t_traj_curr(0),
kp_lin(650.0),
kp_ang(0.0),
kd_lin(30.0),
kd_ang(0.0),
ready_to_start_(false),
traj_computed(false),
debug_mode_(false),
use_jdot_qdot_(true),
use_f_ext_(true),
use_coriolis_(true),
d_ang_max_(100.0),
dw_max_(0.5),
use_xdd_des_(true),
use_mass_sqrt_(false),
elapsed(0),
use_xd_des_(true),
use_ft_sensor_(true),
use_flex_models_(true),
jacobian_solver_type_(WDL_SOLVER),
RTTLWRAbstract(name)
{
//     this->ports()->addPort("PathROS",port_path_ros).doc("");
    this->ports()->addPort("X_curr",port_X_curr).doc("");
    this->ports()->addPort("X_tmp",port_X_tmp).doc("");
    this->ports()->addPort("X_des",port_X_des).doc("");
    this->ports()->addPort("FTData",port_ftdata).doc("The ATI F/T Sensor Input");
    this->addOperation("publishTrajectory",&RttLwrCartCtrl::publishTrajectory,this,RTT::OwnThread);
    this->addOperation("computeTrajectory",&RttLwrCartCtrl::computeTrajectory,this,RTT::OwnThread);
    this->addAttribute("kp_lin",kp_lin);
    this->addAttribute("kp_ang",kp_ang);
    this->addAttribute("kd_lin",kd_lin);
    this->addAttribute("kd_ang",kd_ang);
    this->addAttribute("dw_max",dw_max_);
    this->addAttribute("dx_ang",d_ang_max_);
    this->addAttribute("debug_mode",debug_mode_);
    this->addAttribute("use_jdot_qdot",use_jdot_qdot_);
    this->addAttribute("use_f_ext",use_f_ext_);
    this->addAttribute("use_coriolis",use_coriolis_);
    this->addAttribute("use_xd_des",use_xd_des_);
    this->addAttribute("use_xdd_des",use_xdd_des_);
    this->addAttribute("ReadyToStart",ready_to_start_);
    this->addAttribute("jacobian_solver_type",jacobian_solver_type_);
    this->provides("debug")->addAttribute("elapsed",elapsed);
    this->provides("debug")->addAttribute("WrenchInBase",F_ext);
    this->addAttribute("use_mass_sqrt",use_mass_sqrt_);
    this->addAttribute("use_ft_sensor",use_ft_sensor_);
    this->addAttribute("use_flex_models",use_flex_models_);
}

bool RttLwrCartCtrl::configureHook()
{
    log(Warning) << "Configuring parent" << endlog();
    
    if(false == RTTLWRAbstract::configureHook())
    {
        log(RTT::Fatal) << "Configure parent error" << endlog();
        return false;
    }
    
    log(Warning) << "Configuring ChainJntToJacDotSolver " << endlog();
    jdot_solver.reset(new KDL::ChainJntToJacDotSolver(kdl_chain));
    log(Warning) << "Configuring ChainIkSolverVel_pinv " << endlog();
    pinv_solver.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain));
    log(Warning) << "Configuring ChainIkSolverVel_wdls " << endlog();
    wdls_solver.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain));
    
    log(Warning) << "Getting state" << endlog();
    int cnt = 10;
    while(!updateState() && !cnt--)
    {
        log(Warning) << "Waiting for lwr_fri to publish data " << endlog();
        usleep(5e5);
    }
    
    log(Warning) << "setJointTorqueControlMode" << endlog();
    setJointTorqueControlMode();
    
    log(Warning) << "createStream" << endlog();
    Xd_cmd.setZero();
    X_err.setZero();
    Xd_err.setZero();
    
    port_X_curr.createStream(rtt_roscomm::topic("~"+getName()+"/pos_curr"));
    port_X_corr.createStream(rtt_roscomm::topic("~"+getName()+"/pos_corr"));
    port_X_tmp.createStream(rtt_roscomm::topic("~"+getName()+"/pos_tmp"));
    port_X_des.createStream(rtt_roscomm::topic("~"+getName()+"/pos_des"));
    port_pose_array.createStream(rtt_roscomm::topic("~"+getName()+"/traj_poses"));
    port_path_ros.createStream(rtt_roscomm::topic("~"+getName()+"/cart_traj_des"));
    
    
    qdd_des.resize(kdl_chain.getNrOfJoints());
    mass_kdl.resize(kdl_chain.getNrOfJoints());
    qdd_des_kdl.resize(kdl_chain.getNrOfJoints());
    coriolis.resize(kdl_chain.getNrOfJoints());
    jnt_pos_eigen.resize(kdl_chain.getNrOfJoints());
    jdot.resize(kdl_chain.getNrOfSegments());
    J_ati_base.resize(kdl_chain.getNrOfJoints());
    J_ee_base.resize(kdl_chain.getNrOfJoints());
    // Get initial pose
    getCartesianPosition(cart_pos);
    
    computeTrajectory(0.01,0.05);   
     
    mass_inv.resize(kdl_chain.getNrOfJoints(),kdl_chain.getNrOfJoints());
    return cnt;
}
bool RttLwrCartCtrl::computeTrajectory(const double radius, const double eqradius,const double vmax, const double accmax)
{
    log(Warning) << "Computing Trajectory" << endlog();
    try
    {
        updateState();
        fk_vel_solver->JntToCart(jnt_pos_vel_kdl,frame_vel_des_kdl,kdl_chain.getNrOfSegments());
        frame_des_kdl =  frame_vel_des_kdl.GetFrame();
       
        path = new Path_RoundedComposite(radius,eqradius,new RotationalInterpolation_SingleAxis());
        path->Add(frame_des_kdl);
        path->Add(Frame(Rotation::RPY(99.*deg2rad,        -17.*deg2rad,     -101.*deg2rad),   Vector(-.467,-.448,.576)));
        path->Add(Frame(Rotation::RPY(99.*deg2rad,        -17.*deg2rad,     -101.*deg2rad),   Vector(-.467,-.448,.376)));
        /*path->Add(Frame(Rotation::RPY(88.*deg2rad,        -4.*deg2rad,     -36.*deg2rad),   Vector(-.372,-.527,.505)));
        path->Add(Frame(Rotation::RPY(91.0*deg2rad,         -3.*deg2rad,   -28.*deg2rad), Vector(-.198,-.657,.695)));
        path->Add(Frame(Rotation::RPY(89.0*deg2rad,       -17.*deg2rad,   -32.*deg2rad), Vector(-.219,-.725,.404)));*/
        path->Add(frame_des_kdl);
        // always call Finish() at the end, otherwise the last segment will not be added.
        path->Finish();
        
        velpref = new VelocityProfile_Trap(0.03,0.1);
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

void RttLwrCartCtrl::publishTrajectory()
{
    double dt=static_cast<double>(this->getPeriod());
    nav_msgs::Path path_ros;
    path_ros.header.frame_id = root_link;
    log(Debug) << "Creating Path" << endlog();
    geometry_msgs::PoseArray pose_array;
    for (double t=0.0; t <= traject->Duration(); t+= 0.1) {
            Frame current_pose;
            Twist current_vel,current_acc;
            
            current_pose = traject->Pos(t);
            current_vel = traject->Vel(t);
            current_acc = traject->Acc(t);
                        
            geometry_msgs::Pose pose;
            geometry_msgs::PoseStamped pose_st;
            tf::poseKDLToMsg(current_pose,pose);
            pose_array.poses.push_back(pose);
            pose_st.header.frame_id = path_ros.header.frame_id;
            pose_st.pose = pose;
            path_ros.poses.push_back(pose_st);

    }
    
    pose_array.header.frame_id = path_ros.header.frame_id;
    pose_array.header.stamp = rtt_rosclock::host_now();
    
    port_pose_array.write(pose_array);
    log(Debug) << "Sending Path" << endlog();
    port_path_ros.write(path_ros);
    log(Debug) << "Publishing done" << endlog();
}

void RttLwrCartCtrl::updateHook()
{
    ros::Time t_start = rtt_rosclock::host_now();
    
 
    if(!updateState() || !getMassMatrix(mass) || !traj_computed)
        return;

    Frame X_des,X_mes;
    Twist Xdd_des,Xd_mes,Xd_des;
    
    X_des = traject->Pos(t_traj_curr);
    
    // Fk -> X last frame
    jnt_to_jac_solver->JntToJac(jnt_pos_kdl,J_ati_base,this->seg_names_idx["ati_link"]);
    
    fk_vel_solver->JntToCart(jnt_pos_vel_kdl,tool_in_base_framevel,kdl_chain.getNrOfSegments());
    X_mes =  tool_in_base_framevel.GetFrame();
    Xd_mes = tool_in_base_framevel.GetTwist();
    
    
    // Get ATi FT DATA
    
    if(port_ftdata.read(ft_data) == RTT::NoData && use_ft_sensor_)
    {
        log(RTT::Error) << "No info from sensor" << endlog();
        return;
    }
    tf::wrenchMsgToKDL(ft_data.wrench, ft_wrench_kdl);
    //ft_wrench_kdl = X_mes * ft_wrench_kdl;
    tf::wrenchKDLToEigen(ft_wrench_kdl,F_ext);
    /// Put it in base frame
    
    double estimated_mass = ft_wrench_kdl.force.Norm();
    
    // Flex Models
    Frame X_curr;
    KDL::Frame X_corr = KDL::Frame::Identity();
    if(use_flex_models_)
    {
        jnt_pos_eigen = jnt_pos_kdl.data;
        flex_model_12.compute(jnt_pos_eigen,estimated_mass,corr_cart);
        
        Rotation dr_corr_ = Rotation::Quaternion(corr_cart[3],corr_cart[4],corr_cart[5],corr_cart[6]);
        Twist corr(Vector(corr_cart[0]/1000.0,corr_cart[1]/1000.0,corr_cart[2]/1000.0),dr_corr_.GetRot());

        log(RTT::Debug) << "corr_cart : " << corr_cart.transpose() << endlog();

        X_corr = addDelta(X_mes,corr);
        
        X_curr = X_corr;
    }else{
        X_curr = X_mes;
    }
    Twist d_err = diff(X_curr,X_des);
    
    Frame X_tmp(X_curr);
    X_tmp.M = Rotation::Rot(d_err.rot,d_err.rot.Norm()) * X_tmp.M;
    
    if(1)
    {
        // Ros pub
       publishFrame(port_X_curr,X_mes,root_link);
       publishFrame(port_X_tmp,X_tmp,root_link);
       publishFrame(port_X_des,X_des,root_link);
       publishFrame(port_X_corr,X_corr,root_link);
    }
            
    if(use_xd_des_)
        Xd_des = traject->Vel(t_traj_curr);
    else
        SetToZero(Xd_des);
    Twist dd_err = diff(Xd_mes,Xd_des);
    
    
    
    Xdd_des.vel = kp_lin*d_err.vel + kd_lin*dd_err.vel;
    
    Vector dr = kp_ang*d_err.rot;
    dr(0) = clip(dr(0),-d_ang_max_,d_ang_max_);
    dr(1) = clip(dr(1),-d_ang_max_,d_ang_max_);
    dr(2) = clip(dr(2),-d_ang_max_,d_ang_max_);
    
    Xdd_des.rot = dr + kd_ang*(dd_err.rot);
    
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
            if(use_mass_sqrt_)
                wdls_solver->setWeightJS(mass.sqrt());
            else
                wdls_solver->setWeightJS(mass.inverse());
            
            ret = wdls_solver->CartToJnt(jnt_pos_kdl,Xdd_des,qdd_des_kdl);
            break;
        default:
            log(RTT::Error) << "Invalid mode" << endlog();
            this->error();
            
    }
    
    
    id_dyn_solver->JntToCoriolis(jnt_pos_kdl,jnt_vel_kdl,coriolis_kdl);
    
    jnt_trq_cmd = mass*qdd_des_kdl.data;
    if(use_f_ext_)
        jnt_trq_cmd += J_ati_base.data.transpose() * F_ext;
    if(use_coriolis_)
        jnt_trq_cmd += coriolis_kdl.data;
    
    log(Debug) <<"Sensor "<< ft_wrench_kdl << endlog();
    log(Debug) << "trqcmd " << jnt_trq_cmd.transpose() << endlog();
    
    if(!debug_mode_)
        if(!isCommandMode())
            return;
        
        sendJointTorque(jnt_trq_cmd);
    
    // Incremente traj
    if(isReadyToStart()){
        if( t_traj_curr <= traject->Duration())
            t_traj_curr += static_cast<double>(this->getPeriod());
        else
            t_traj_curr = 0.0;
    }
    ros::Duration t_elapsed = rtt_rosclock::host_now() - t_start;
    elapsed = t_elapsed.toSec();
}

}
