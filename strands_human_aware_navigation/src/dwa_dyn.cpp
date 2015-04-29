#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

bool setup_ = false;
dwa_local_planner::DWAPlannerConfig default_config_;


void callback(dwa_local_planner::DWAPlannerConfig &config, uint32_t level) {
    if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
    }
    if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
    }

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf, human_conf;

    // Incredibly terrible C++ dynamic reconfigure client emulation...
    bool_param.name   = "prune_plan";              bool_param.value = config.prune_plan;                conf.bools.push_back(bool_param);
    bool_param.name   = "restore_defaults";        bool_param.value = config.restore_defaults;          conf.bools.push_back(bool_param);
    bool_param.name   = "use_dwa";                 bool_param.value = config.use_dwa;                   conf.bools.push_back(bool_param);

    int_param.name    = "vth_samples";             int_param.value = config.vth_samples;                conf.ints.push_back(int_param);
    int_param.name    = "vx_samples";              int_param.value = config.vx_samples;                 conf.ints.push_back(int_param);
    int_param.name    = "vy_samples";              int_param.value = config.vy_samples;                 conf.ints.push_back(int_param);

    double_param.name = "acc_limit_trans";         double_param.value = config.acc_limit_trans;         conf.doubles.push_back(double_param);
    double_param.name = "acc_lim_theta";           double_param.value = config.acc_lim_theta;           conf.doubles.push_back(double_param);
    double_param.name = "acc_lim_x";               double_param.value = config.acc_lim_x;               conf.doubles.push_back(double_param);
    double_param.name = "acc_lim_y";               double_param.value = config.acc_lim_y;               conf.doubles.push_back(double_param);
    double_param.name = "angular_sim_granularity"; double_param.value = config.angular_sim_granularity; conf.doubles.push_back(double_param);
    double_param.name = "forward_point_distance";  double_param.value = config.forward_point_distance;  conf.doubles.push_back(double_param);
    double_param.name = "goal_distance_bias";      double_param.value = config.goal_distance_bias;      conf.doubles.push_back(double_param);
    double_param.name = "max_rot_vel";             double_param.value = config.max_rot_vel;             conf.doubles.push_back(double_param); human_conf.doubles.push_back(double_param);
    double_param.name = "max_scaling_factor";      double_param.value = config.max_scaling_factor;      conf.doubles.push_back(double_param);
    double_param.name = "max_trans_vel";           double_param.value = config.max_trans_vel;           conf.doubles.push_back(double_param); human_conf.doubles.push_back(double_param);
    double_param.name = "max_vel_x";               double_param.value = config.max_vel_x;               conf.doubles.push_back(double_param); human_conf.doubles.push_back(double_param);
    double_param.name = "max_vel_y";               double_param.value = config.max_vel_y;               conf.doubles.push_back(double_param);
    double_param.name = "min_rot_vel";             double_param.value = config.min_rot_vel;             conf.doubles.push_back(double_param);
    double_param.name = "min_trans_vel";           double_param.value = config.min_trans_vel;           conf.doubles.push_back(double_param);
    double_param.name = "min_vel_x";               double_param.value = config.min_vel_x;               conf.doubles.push_back(double_param);
    double_param.name = "min_vel_y";               double_param.value = config.min_vel_y;               conf.doubles.push_back(double_param);
    double_param.name = "occdist_scale";           double_param.value = config.occdist_scale;           conf.doubles.push_back(double_param);
    double_param.name = "oscillation_reset_angle"; double_param.value = config.oscillation_reset_angle; conf.doubles.push_back(double_param);
    double_param.name = "oscillation_reset_dist";  double_param.value = config.oscillation_reset_dist;  conf.doubles.push_back(double_param);
    double_param.name = "path_distance_bias";      double_param.value = config.path_distance_bias;      conf.doubles.push_back(double_param);
    double_param.name = "rot_stopped_vel";         double_param.value = config.rot_stopped_vel;         conf.doubles.push_back(double_param);
    double_param.name = "scaling_speed";           double_param.value = config.scaling_speed;           conf.doubles.push_back(double_param);
    double_param.name = "sim_granularity";         double_param.value = config.sim_granularity;         conf.doubles.push_back(double_param);
    double_param.name = "sim_time";                double_param.value = config.sim_time;                conf.doubles.push_back(double_param);
    double_param.name = "stop_time_buffer";        double_param.value = config.stop_time_buffer;        conf.doubles.push_back(double_param);
    double_param.name = "trans_stopped_vel";       double_param.value = config.trans_stopped_vel;       conf.doubles.push_back(double_param);
    double_param.name = "xy_goal_tolerance";       double_param.value = config.xy_goal_tolerance;       conf.doubles.push_back(double_param);
    double_param.name = "yaw_goal_tolerance";      double_param.value = config.yaw_goal_tolerance;      conf.doubles.push_back(double_param);

    srv_req.config = conf;
    ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
    srv_req.config = human_conf;
    ros::service::call("/human_aware_navigation/set_parameters", srv_req, srv_resp);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "DWAPlannerROS_dyn_wrapper");

    dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig> server(ros::NodeHandle("/human_aware_navigation/DWAPlannerROS"));
    dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}
