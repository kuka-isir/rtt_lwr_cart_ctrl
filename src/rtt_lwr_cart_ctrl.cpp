#include "rtt_lwr_cart_ctrl/rtt_lwr_cart_ctrl.hpp"
#include "eigen_conversions/eigen_msg.h"

#include <unistd.h>

namespace lwr{
using namespace RTT;
using namespace KDL;

RttLwrCartCtrl::RttLwrCartCtrl(const std::string& name):
t_traj_curr(0),
kp_lin(1000.0),
kp_ang(650.0),
kd_lin(30.0),
kd_ang(20.0),
dw_max_(0.05),
ready_to_start_(false),
traj_computed(false),
debug_mode_(false),
use_jdot_qdot_(true),
use_f_ext_(true),
use_coriolis_(true),
d_ang_max_(100.0),
use_xdd_des_(true),
use_mass_sqrt_(false),
elapsed(0),
use_xd_des_(true),
use_ft_sensor_(true),
use_flex_models_(false),
use_damping_(true),
use_nso_(false),
damping_(10.0),
kp_ref_(100.0),
jacobian_solver_type_(WDL_SOLVER),
RTTLWRAbstract(name)
{
    this->ports()->addPort("FTData",port_ftdata).doc("The ATI F/T Sensor Input");
    this->addOperation("publishTrajectory",&RttLwrCartCtrl::publishTrajectory,this,RTT::OwnThread);
    this->addOperation("computeTrajectory",&RttLwrCartCtrl::computeTrajectory,this,RTT::OwnThread);
    this->addOperation("setGains",&RttLwrCartCtrl::setGains,this,RTT::OwnThread);
    this->addProperty("kp_lin",kp_lin);
    this->addProperty("kp_ang",kp_ang);
    this->addProperty("kd_lin",kd_lin);
    this->addProperty("kd_ang",kd_ang);
    this->addProperty("dw_max",dw_max_);
    this->addProperty("use_nso",use_nso_);
    this->addProperty("kp_ref",kp_ref_);
    this->addProperty("kt",kt_);
    this->addProperty("dx_ang_max",d_ang_max_);
    this->addProperty("debug_mode",debug_mode_);
    this->addProperty("use_jdot_qdot",use_jdot_qdot_);
    this->addProperty("use_f_ext",use_f_ext_);
    this->addProperty("use_coriolis",use_coriolis_);
    this->addProperty("use_xd_des",use_xd_des_);
    this->addProperty("use_damping",use_damping_);
    this->addProperty("damping",damping_);
    this->addProperty("use_xdd_des",use_xdd_des_);
    this->addProperty("ReadyToStart",ready_to_start_);
    this->addProperty("jacobian_solver_type",jacobian_solver_type_);
    this->provides("debug")->addAttribute("elapsed",elapsed);
    this->provides("debug")->addAttribute("WrenchInBase",F_ext);
    this->addProperty("use_mass_sqrt",use_mass_sqrt_);
    this->addProperty("use_ft_sensor",use_ft_sensor_);
    this->addProperty("use_flex_models",use_flex_models_);
}

bool RttLwrCartCtrl::configureHook()
{
    log(Warning) << "Configuring parent" << endlog();
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
          this->getProvider<rtt_rosparam::ROSParam>("rosparam");
    if(rosparam) {
        const RTT::PropertyBag::Properties &properties =  this->properties()->getProperties();

        for(RTT::PropertyBag::Properties::const_iterator it = properties.begin();
            it != properties.end();
            ++it)
        {
            log(Info) << "RosParams :  "<< endlog();
            if(rosparam->getParam(getName() +"/"+(*it)->getName(),(*it)->getName()))
                log(Info) << getName() +"/"+(*it)->getName() << " => "<< this->getProperty((*it)->getName())<< endlog();
        }
    }
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
    
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain,js_cmd);
    
    port_X_curr.createStream(rtt_roscomm::topic("~"+getName()+"/pos_curr"));
    port_X_corr.createStream(rtt_roscomm::topic("~"+getName()+"/pos_corr"));
    port_X_tmp.createStream(rtt_roscomm::topic("~"+getName()+"/pos_tmp"));
    port_X_des.createStream(rtt_roscomm::topic("~"+getName()+"/pos_des"));
    port_pose_array.createStream(rtt_roscomm::topic("~"+getName()+"/traj_poses"));
    port_path_ros.createStream(rtt_roscomm::topic("~"+getName()+"/cart_traj_des"));
    port_js.createStream(rtt_roscomm::topic("~"+getName()+"/joint_cmd"));
    port_wrench_world.createStream(rtt_roscomm::topic("~"+getName()+"/wrench_world"));
    
    qdd_des.resize(kdl_chain.getNrOfJoints());
    mass_kdl.resize(kdl_chain.getNrOfJoints());
    qdd_des_kdl.resize(kdl_chain.getNrOfJoints());
    coriolis.resize(kdl_chain.getNrOfJoints());
    jnt_pos_eigen.resize(kdl_chain.getNrOfJoints());
    jdot.resize(kdl_chain.getNrOfSegments());
    J_ati_base.resize(kdl_chain.getNrOfJoints());
    J_ee_base.resize(kdl_chain.getNrOfJoints());
    jnt_zero.resize(kdl_chain.getNrOfJoints());
    SetToZero(jnt_zero);
    mass_kdl.resize(kdl_chain.getNrOfJoints());
    kt_.resize(kdl_chain.getNrOfJoints());
    kt_.setConstant(1.0);
    
    I.resize(kdl_chain.getNrOfJoints(),kdl_chain.getNrOfJoints());
    I.setIdentity();
    
    J_pm.resize(kdl_chain.getNrOfJoints(),6);
    J_pm.setZero();
    
    Jt.resize(kdl_chain.getNrOfJoints(),6);
    Jt.setZero();
    
    // Get initial pose
    
    jnt_pos_ref_ = jnt_pos;
    
    getCartesianPosition(cart_pos);
    
    computeTrajectory(0.01,0.05);   
     
    mass_inv.resize(kdl_chain.getNrOfJoints(),kdl_chain.getNrOfJoints());
    
    
    log(Warning) << "IDX for the ft sensor is "<< this->seg_names_idx["ati_link"] << endlog();
    
    
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
        /*path->Add(Frame(Rotation::RPY(99.*deg2rad,        -17.*deg2rad,     -101.*deg2rad),   Vector(-.467,-.448,.576)));
        path->Add(Frame(Rotation::RPY(99.*deg2rad,        -17.*deg2rad,     -101.*deg2rad),   Vector(-.467,-.448,.376)));*/
        path->Add(Frame(frame_des_kdl.M,   Vector(-.467,-.448,.576)));
        path->Add(Frame(frame_des_kdl.M,   Vector(-.467,-.448,.376)));
        /*path->Add(Frame(Rotation::RPY(88.*deg2rad,        -4.*deg2rad,     -36.*deg2rad),   Vector(-.372,-.527,.505)));
        path->Add(Frame(Rotation::RPY(91.0*deg2rad,         -3.*deg2rad,   -28.*deg2rad), Vector(-.198,-.657,.695)));
        path->Add(Frame(Rotation::RPY(89.0*deg2rad,       -17.*deg2rad,   -32.*deg2rad), Vector(-.219,-.725,.404)));*/
        path->Add(frame_des_kdl);
        // always call Finish() at the end, otherwise the last segment will not be added.
        path->Finish();
        
        velpref = new VelocityProfile_Trap(vmax,accmax);
        velpref->SetProfile(0,path->PathLength());  
        traject = new Trajectory_Segment(path, velpref);
        
        ctraject = new Trajectory_Composite();
        ctraject->Add(traject);
        ctraject->Add(new Trajectory_Stationary(3.0,frame_des_kdl));
        
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
void RttLwrCartCtrl::setGains(const double kp_lin, const double kp_ang, const double kd_lin, const double kd_ang)
{
    this->kp_lin = kp_lin;
    this->kp_ang = kp_ang;
    this->kd_lin = kd_lin;
    this->kd_ang = kd_ang;
}

void RttLwrCartCtrl::updateHook()
{
    ros::Time t_start = rtt_rosclock::host_now();
 
    if(!updateState() || !getMassMatrix(mass) || !traj_computed)
        return;

    log(RTT::Debug) << "Mass matrix : \n        " << mass << endlog();
    
    id_dyn_solver->JntToMass(jnt_pos_kdl,mass_kdl);
    
    log(RTT::Debug) << "Mass matrix KDL : \n        " << mass_kdl.data << endlog();
    
    Frame X_des,X_mes;
    Twist Xdd_des,Xd_mes,Xd_des;
    
    if(t_traj_curr == 0)
        X_des = frame_des_kdl;
    else
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
    
    // Flex Models
    Frame X_curr;
    KDL::Frame X_corr = KDL::Frame::Identity();
    if(use_flex_models_)
    {
        fk_vel_solver->JntToCart(jnt_pos_vel_kdl,tool_in_base_framevel,seg_names_idx["ati_link"]);
    
        KDL::Wrench w = tool_in_base_framevel.GetFrame().M * ft_wrench_kdl;

        w_msg.header.frame_id = root_link;
        w_msg.header.stamp = rtt_rosclock::host_now();
        tf::wrenchKDLToMsg(w,w_msg.wrench);
        port_wrench_world.write(w_msg);
        
        
        jnt_pos_eigen = jnt_pos_kdl.data;
        tf::wrenchKDLToEigen(w,F_ext);
        flex_model_42.compute(jnt_pos_eigen,F_ext,corr_cart);
        
        Rotation dr_corr_ = Rotation::Quaternion(corr_cart[3],corr_cart[4],corr_cart[5],corr_cart[6]);
        Twist corr(Vector(corr_cart[0]/1000.0,corr_cart[1]/1000.0,corr_cart[2]/1000.0),dr_corr_.GetRot());

        log(RTT::Debug) << "corr_cart : " << corr_cart.transpose() << endlog();

        X_corr = addDelta(X_mes,-corr);
            
        X_curr = X_corr;
    }else{
        X_curr = X_mes;
        X_corr = X_mes;
    }
    Twist d_err = diff(X_curr,X_des);
    
    if(d_err.rot.Norm() > dw_max_)
    {
        d_err.rot.Normalize();
        d_err.rot = d_err.rot*dw_max_;
    }
    
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
    
        
    log(RTT::Debug) << "dr : " << dr << endlog();
    
    Xdd_des.rot = dr + kd_ang*(dd_err.rot);
    
    // Jdot*qdot
    jdot_solver->JntToJacDot(jnt_pos_vel_kdl,jdot_qdot,kdl_chain.getNrOfSegments());
    
    if(use_jdot_qdot_)
        Xdd_des -= jdot_qdot;
    
    log(RTT::Debug) << "dr : " << dr << endlog();
    
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
        case JACOBIAN_TRANSPOSE:
            break;
        default:
            log(RTT::Error) << "Invalid mode" << endlog();
            this->error();
            
    }
    
    if(use_nso_){
        if(jacobian_solver_type_ ==  JACOBIAN_TRANSPOSE)
            mass_inv = mass.inverse();
        else
           mass_inv = (mass*mass).inverse(); 
        
        log(RTT::Debug) << "mass_inv matrix : \n" << mass_inv << endlog();
        
        J_pm = mass_inv * J_ati_base.data.transpose() * (J_ati_base.data * mass_inv * J_ati_base.data.transpose()).inverse();

       ////// qdd_des_kdl.data += (I - J_pm*J_ati_base.data) * (- damping_ * jnt_vel);
    }
    
    
    id_dyn_solver->JntToCoriolis(jnt_pos_kdl,jnt_vel_kdl,coriolis_kdl);
    
    Eigen::Matrix<double,6,1> xdd_des;
    tf::twistKDLToEigen(Xdd_des,xdd_des);
    
    // Jt 
    if(jacobian_solver_type_ ==  JACOBIAN_TRANSPOSE)
        jnt_trq_cmd = J_ati_base.data.transpose() * (xdd_des);
    else
        jnt_trq_cmd = mass*qdd_des_kdl.data;
    
    
    if(use_nso_)
    {
        if(jacobian_solver_type_ ==  JACOBIAN_TRANSPOSE)
            jnt_trq_cmd += (I - J_ati_base.data.transpose() * J_pm.transpose())  * (kp_ref_*(jnt_pos_ref_ - jnt_pos) - damping_ * jnt_vel);
        else
           jnt_trq_cmd += (I - J_ati_base.data.transpose() * J_pm.transpose())  * mass * (kp_ref_*(jnt_pos_ref_ - jnt_pos) - damping_ * jnt_vel); 
    }
    
    
    if(use_f_ext_)
    {
        // Get the Wrench in the last segment's frame
        f_ext_kdl[this->seg_names_idx["ati_link"]] = ft_wrench_kdl;
        // Get The full dynamics with external forces
        id_rne_solver->CartToJnt(jnt_pos_kdl,jnt_zero,jnt_zero,f_ext_kdl,jnt_trq_kdl);
        // Get Joint Gravity torque
        id_dyn_solver->JntToGravity(jnt_pos_kdl,gravity_kdl);
        // remove G(q)
        jnt_trq_cmd += jnt_trq_kdl.data - gravity_kdl.data;
    }
    
    if(use_coriolis_)
        jnt_trq_cmd += coriolis_kdl.data;
    
    if(use_damping_)
        jnt_trq_cmd -= damping_*jnt_vel;
    

    log(Debug) <<"Sensor "<< ft_wrench_kdl << endlog();
    log(Debug) << "trqcmd " << jnt_trq_cmd.transpose() << endlog();
    
    if(!debug_mode_)
        if(!isCommandMode())
            return;
        
    js_cmd.header.stamp = rtt_rosclock::host_now();
    Eigen::Map<Eigen::VectorXd>(js_cmd.velocity.data(),kdl_chain.getNrOfJoints()) = qdd_des_kdl.data;
    Eigen::Map<Eigen::VectorXd>(js_cmd.effort.data(),kdl_chain.getNrOfJoints()) = jnt_trq_cmd;    
    port_js.write(js_cmd);
        
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
