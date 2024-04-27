#include <fstream>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mbf_advanced/mbf_circle_client2.h>



#include <string>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

///
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
///

class Charge30 : public BT::SyncActionNode
    {
    public:
        Charge30(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
            , distance_(30.0) // 默认移动距离为 30 厘米
        {
            nh_.reset(new ros::NodeHandle);
            cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<double>("distance", "Distance to move (in cm)") };
        }

        BT::NodeStatus tick() override
        {
            // 获取距离输入端口的值，如果存在
            if (auto distance = getInput<double>("distance"))
            {
                distance_ = *distance / 100.0; // 更新距离为输入值，并将单位转换为米
            }

            // 构造机器人移动命令
            geometry_msgs::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = -0.1; // 线速度为 0.1 米/秒
            cmd_vel_msg.angular.z = 0.0; // 角速度为 0
            ros::Time start_time = ros::Time::now();
            double distance_travelled = 0.0;
            ros::Rate loop_rate(10); // 控制机器人移动的频率为 10 Hz

            // 发布机器人移动命令，并计算机器人移动的距离
            while (distance_travelled < distance_ && ros::ok())
            {
                cmd_vel_pub_.publish(cmd_vel_msg);
                distance_travelled = 0.1 * (ros::Time::now() - start_time).toSec();
                loop_rate.sleep();
            }

            // 停止机器人移动
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_pub_.publish(cmd_vel_msg);
            ROS_INFO("Robot has back travelled %f meters.", distance_travelled);
            // 返回成功
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::unique_ptr<ros::NodeHandle> nh_;
        ros::Publisher cmd_vel_pub_;
        double distance_; // 移动的距离
    };


class CheckBattery : public BT::SyncActionNode
{
public:
    explicit CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        // 從參數伺服器讀取檔案路徑
        if (!getInput<std::string>("file_path", file_path_))
        {
            throw BT::RuntimeError("missing required input [file_path]");
        }

        // 從參數伺服器讀取低電量閾值
        if (!getInput<float>("low_threshold", low_threshold_))
        {
            throw BT::RuntimeError("missing required input [low_threshold]");
        }
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("file_path", "Path to battery level file"),
                 BT::InputPort<float>("low_threshold", "Low battery level threshold"),
                 BT::OutputPort<float>("battery_level", "Current battery level (0-100%)") };
    }

    BT::NodeStatus tick() override
    {
        std::ifstream file(file_path_);
        if (!file.is_open())
        {
            ROS_ERROR_STREAM("Failed to open file: " << file_path_);
            return BT::NodeStatus::FAILURE;
        }

        std::string line;
        if (!std::getline(file, line))
        {
            ROS_ERROR_STREAM("Failed to read battery level from file: " << file_path_);
            return BT::NodeStatus::FAILURE;
        }

        std::istringstream ss(line);
        float voltage, percentage;
        char delimiter;
        if (!(ss >> voltage >> delimiter >> percentage) || delimiter != ',')
        {
            ROS_ERROR_STREAM("Invalid battery level format in file: " << file_path_);
            return BT::NodeStatus::FAILURE;
        }

        float battery_level = percentage;
        setOutput("battery_level", battery_level);

        if (battery_level < low_threshold_)
        {
            ROS_WARN_STREAM("Battery level is low: " << battery_level << "%");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO_STREAM("Battery level is sufficient: " << battery_level << "%");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    std::string file_path_;
    float low_threshold_;
};




class GoToPose : public BT::SyncActionNode
{
public:
    explicit GoToPose(const std::string& name)
      : BT::SyncActionNode(name, {})
      , mbfclient_{}
    { }

    void attachMBFClient(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient)
    {
        mbfclient_ = mbfclient;
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO_STREAM("BT: " << this->name());

        if (mbfclient_)
        {

            ROS_INFO_STREAM("BT: " << this->name());
            auto r=mbfclient_->move_to_goal();
            if (r->outcome  == mbf_msgs::MoveBaseResult::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
           }

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
};


class GoToPoseCharge : public BT::SyncActionNode
{
public:
    explicit GoToPoseCharge(const std::string& name)
      : BT::SyncActionNode(name, {})
      , mbfclient_c{}
    { }

    void attachMBFClientcharge(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_charge)
    {
        mbfclient_c = mbfclient_charge;
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO_STREAM("BT: " << this->name());

        if (mbfclient_c)
        {

            ROS_INFO_STREAM("BT: " << this->name());
            auto r=mbfclient_c->move_to_goal();
            if (r->outcome  == mbf_msgs::MoveBaseResult::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
           }

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_c;
};




class LookForObject : public BT::SyncActionNode
{
public:
    explicit LookForObject(const std::string& name)
               : BT::SyncActionNode(name, {})
               , mbfclient_{}
               , mbfclient_c{}

    { }

    void attachMBFClient(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient)
    {
        mbfclient_ = mbfclient;
    }

    void attachMBFClientcharge(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_charge)
    {
        mbfclient_c = mbfclient_charge;
    }



    BT::NodeStatus tick() override
    {
        ROS_INFO_STREAM("BT: " << this->name());

        if (mbfclient_ || mbfclient_c)
        {
            ROS_INFO_STREAM("            welldone got to the next location!!        ");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_c;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbf_bt_v1");

    ros::NodeHandle n;


    auto mbfclient = std::make_shared<mbf_advanced::MBFCircleClient>(std::move(mbf_advanced::loadPoseGoals(POSE_PATH2)));
    auto mbfclient_charge = std::make_shared<mbf_advanced::MBFCircleClient>(std::move(mbf_advanced::loadPoseGoals(POSE_PATHCHARGE)));




    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<Charge30>("Charge30");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<GoToPose>("GoToPose");
    factory.registerNodeType<GoToPoseCharge>("GoToPoseCharge");
    factory.registerNodeType<LookForObject>("LookForObject");


    auto tree = factory.createTreeFromFile(BT_XML_PATH2);

    for( auto& node: tree.nodes)
    {


        if( auto gotopose = dynamic_cast<GoToPose*>( node.get() ))
        {
            gotopose->attachMBFClient(mbfclient);
          //  std::cout<< AW<<"1\n";
          //  std::cout<< dynamic_cast<GoToPose*>( node.get() )<<"1\n";

        }

        if( auto gotoposecharge = dynamic_cast<GoToPoseCharge*>( node.get() ))
        {
            gotoposecharge->attachMBFClientcharge(mbfclient_charge);
          //  std::cout<< AW<<"1\n";
          //  std::cout<< dynamic_cast<GoToPose*>( node.get() )<<"1\n";

        }

        if( auto lookforobject = dynamic_cast<LookForObject*>( node.get() ))
        {
            lookforobject->attachMBFClient(mbfclient);
            lookforobject->attachMBFClientcharge(mbfclient_charge);
           // std::cout<< AW<<"2\n";
          //  std::cout<< dynamic_cast<LookForObject*>( node.get() )<<"1\n";

        }

    }

    BT::PublisherZMQ publisher_zmq(tree);
    int c=0;
    std::cout<< AW<<" last line"<<std::endl;
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        if (status!=BT::NodeStatus::RUNNING)
            c++;
        std::cout<<"count : "<<c<<", "<<status<<std::endl;
        //ros::Duration(5).sleep();
    }


    return 0;
}
