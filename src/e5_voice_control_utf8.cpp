#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <map>
#include <functional>
#include <thread>
#include <atomic>
#include "geometry_msgs/Twist.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include "aubo_grasp/graspMessage.h"
#include "aubo_grasp/followingMessage.h"
#include "aubo_grasp/placingMessage.h"

//注意：以下代码中的中文字符已经转换为UTF-8编码
geometry_msgs::Twist twist;//可视化节点话题发布内容
ros::Publisher cmd_vel_pub;//可视化节点话题发布者
std_msgs::String string_msg;//语音识别结果
ros::Publisher tts_text_pub;//语音合成发布器

//抓取相关的消息和发布器
aubo_grasp::graspMessage grasp_msg;// 抓取消息
ros::Publisher grasp_pub;//抓取发布器
aubo_grasp::followingMessage following_msg;// 跟随物体消息
ros::Publisher following_pub;//跟随物体发布器
aubo_grasp::placingMessage placing_msg;// 放置物体消息
ros::Publisher placing_pub;//放置物体发布器

//全局变量定义
int cmd_type = -1; //当前机器人的控制状态
int joint_num = -1; //关节编号
double angle3 = 90.0; //关节3角度
double angle4 = 0.0; //关节4角度
double gripper_width = 0.0; //夹爪宽度
double rotation_angle = 0.0; //旋转角度
bool in_sync_mode = false; //同步模式标志
bool syncModeActive = false; //同步模式状态变化
std::atomic<bool> gripperThreadRunning(true); // 定义一个原子布尔值，表示夹爪线程是否运行

// 命令类型枚举
enum CommandType {
    EXIT = 0, //退出
    JOINT_ROTATE = 1, //关节旋转
    INITIAL_POSE = 2, //初始姿态
    DIRECT_GRASP = 3, //直接抓取
    GRASP_ITEM = 4, //抓取物品
    SYNC_MODE = 5, //同步模式
    EXIT_SYNC = 6, //退出同步
    PLACING_POSE = 7, //放置初始姿态
    PLACING_FINAL_POSE = 8, //放置最终姿态
    GRIPPER_CLOSE = 11, //夹爪关闭
    GRIPPER_OPEN = 12, //夹爪张开
    GRIPPER_ACTIVE = 13, //夹爪激活
    FOLLOWING_OBJECT = 14, //跟随物体
    PLACING_OBJECT = 15, //放置物体
    EXIT_FOLLOWING = 16, //退出跟随物体
};

// 定义命令处理函数
typedef std::function<void(const std::string&)> CommandHandler;
void handleExit(const std::string& dataString);
void handleJointRotate(const std::string& dataString);
void handleInitialPose(const std::string& dataString);
void handleDirectGrasp(const std::string& dataString);
void handleGraspItem(const std::string& dataString);
void handleSyncMode(const std::string& dataString);
void handleExitSync(const std::string& dataString);
void handleGripperClose(const std::string& dataString);
void handleGripperOpen(const std::string& dataString);
void handleGripperActive(const std::string& dataString);
void handleFollowingObject(const std::string& dataString);
void handleExitFollowingObject(const std::string& dataString);
void handlePlacingObject(const std::string& dataString);
void handlePlacingPose(const std::string& dataString);
void handlePlacingFinalPose(const std::string& dataString);

// 存储关键词和对应的命令处理函数及命令类型
std::map<std::string, std::pair<CommandHandler, CommandType>> commandMap = {
    {"再见", {handleExit, EXIT}},
    {"旋转", {handleJointRotate, JOINT_ROTATE}},
    {"初始", {handleInitialPose, INITIAL_POSE}},
    {"初始姿态", {handleInitialPose, INITIAL_POSE}},
    {"直接抓", {handleDirectGrasp, DIRECT_GRASP}},
    {"随机抓", {handleDirectGrasp, DIRECT_GRASP}},
    {"抓起", {handleGraspItem, GRASP_ITEM}},
    {"抓取", {handleGraspItem, GRASP_ITEM}},
    {"同步模式", {handleSyncMode, SYNC_MODE}},
    {"同步姿态", {handleSyncMode, SYNC_MODE}},
    {"退出同步", {handleExitSync, EXIT_SYNC}},
    {"测试姿态", {handlePlacingPose, PLACING_POSE}},
    {"最终姿态", {handlePlacingFinalPose, PLACING_FINAL_POSE}},
    {"关闭爪", {handleGripperClose, GRIPPER_CLOSE}},
    {"闭合夹爪", {handleGripperClose, GRIPPER_CLOSE}},
    {"张开爪", {handleGripperOpen, GRIPPER_OPEN}},
    {"张开转", {handleGripperOpen, GRIPPER_OPEN}},
    {"张开夹爪", {handleGripperOpen, GRIPPER_OPEN}},
    {"激活夹爪", {handleGripperActive, GRIPPER_ACTIVE}},
    {"激活甲状", {handleGripperActive, GRIPPER_ACTIVE}},
    {"跟随", {handleFollowingObject, FOLLOWING_OBJECT}},
    {"跟踪", {handleFollowingObject, FOLLOWING_OBJECT}},
    {"退出跟", {handleExitFollowingObject, EXIT_FOLLOWING}},
    {"放", {handlePlacingObject, PLACING_OBJECT}},
};

//中文数字映射
std::map<std::string, int> chineseNumberMap = {
    {"一", 1}, {"衣", 1},{"要", 1}, {"医", 1}, {"腰", 1}, {"二", 2}, {"三", 3}, 
    {"四", 4}, {"五", 5}, {"六", 6}, {"七", 7}, {"八", 8},{"九", 9}, {"十", 10},
    {"十一", 11}, {"十二", 12}, {"十三", 13}, {"十四", 14}, {"十五", 15},
    {"十六", 16}, {"十七", 17}, {"十八", 18}, {"十九", 19}, {"二十", 20}
};

//同步模式回调函数
void armcallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 5) {
        //打印4个数据
        //ROS_INFO("Received data: %f", msg->data[2]);
        double angle1 = msg->data[0];
        //设置关节4的角度范围为-40到90，小于50为-40，大于180为90
        if(angle1 < 50) angle4 = -40;
        else if(angle1 > 180) angle4 = 90;
        else angle4 = angle1 -90;
        //设置关节3的角度范围为180 - angle2，小于65为115，大于150为30
        double angle2 = msg->data[1];
        if(angle2 < 65) angle3 = 115;
        else if(angle2 > 150) angle3 = 30;
        else angle3 = 180 - angle2;
        //设置夹爪宽度范围为6到45，小于6为1.0，大于45为0.0
        double thumb_index_data = msg->data[2];
        if(thumb_index_data < 6) gripper_width = 1.0;
        else if(thumb_index_data > 45) gripper_width = 0.0;
        else gripper_width = 1.0 - (thumb_index_data - 6) / 39.0;
        //同步模式标志，如果msg->data[4]为1，则为true，否则为false
        in_sync_mode = msg->data[4] == 1;
        // 如果msg->data[4]为0且syncModeActive为true，则执行一次初始姿态的命令
        if (msg->data[4] == 0 && syncModeActive) {
            // 执行一次返回初始姿态的命令
            cmd_type = 6;
            syncModeActive = false; // 设置标志位为已执行
        }
    } else {
        ROS_WARN("Received incomplete data.");
    }
}

// 语音识别回调函数
void iattextCallback(const std_msgs::String::ConstPtr& msg) {
    const char* text;
    std::string dataString = msg->data;
    //指令识别
    for (const auto& pair : commandMap) {
        if (dataString.find(pair.first) != std::string::npos) {
            pair.second.first(dataString);
            //ROS_INFO("Received command: %s", pair.first.c_str());
            cmd_type = pair.second.second;
            return;
        }
    }
    //未识别的指令语音合成发布
    ROS_WARN("Unrecognized command");
    cmd_type = -1;
    text = "未识别的命令";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
}

//获取关节编号
int getJointNumber(const std::string& str) {
    for (const auto& pair : chineseNumberMap) {
        if (str.find(pair.first) != std::string::npos) {
            //如果pair.second大于6或小于1，则返回-1
            if (pair.second > 6 || pair.second < 1) return -1;// 未识别的关节编号
            else return pair.second;
        }
    }
    return -1;  // 未识别的关节编号
}

//获取旋转角度
double getRotationAngle(const std::string& str) {
    if (str.empty()) return 0.0;
    int angle = 0;
    for (const auto& pair : chineseNumberMap) {
        if (str.find(pair.first) != std::string::npos) {
            angle = pair.second;
            break;
        }
    }
    if (angle == 0) {
        try {
            return std::stod(str);
        } catch (const std::exception& e) {
            ROS_WARN("Failed to parse rotation angle: %s", e.what());
        }
    }
    return angle;
}

// 处理退出指令
void handleExit(const std::string& dataString) {
    const char* text;
    text = "已退出语音控制";
}

// 处理关节旋转指令
void handleJointRotate(const std::string& dataString) {
    const char* text;
    //打印识别结果
    std::string joint_num_str;
    std::string rotation_angle_str;
    size_t pos = dataString.find("旋转");
    //分割joint_num_str为关节编号 rotation_angle_str为旋转角度
    joint_num_str = dataString.substr(0, pos);
    rotation_angle_str = dataString.substr(pos + 6);
    //获取关节编号和旋转角度
    joint_num = getJointNumber(joint_num_str);
    rotation_angle = getRotationAngle(rotation_angle_str);
    if (joint_num != -1 && rotation_angle != 0.0) {
        //拼接语音指令并发布到语音合成节点
        ROS_INFO("Received command: joint %d rotate %.1f degrees", joint_num, rotation_angle);
        char forwordString[40];
        snprintf(forwordString, sizeof(forwordString), "关节%d旋转%.1f度", joint_num, rotation_angle);
        text = forwordString;
        string_msg.data = text;
        tts_text_pub.publish(string_msg);
    } else {
        ROS_WARN("Unrecognized joint number or rotation angle");
        text = "未识别的关节编号或旋转角度";  
        string_msg.data = text;
        tts_text_pub.publish(string_msg);      
    }
}

// 处理初始姿态指令
void handleInitialPose(const std::string& dataString) {
    const char* text;
    text = "设置为初始姿态";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: set to initial pose");
}

// 处理直接抓取/直接取物品指令
void handleDirectGrasp(const std::string& dataString) {
    const char* text;
    text = "开始随机抓取";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: direct grasp");
}

// 处理抓取物品指令
void handleGraspItem(const std::string& dataString) {
    const char* text;
    //判断dataString是否包含框、瓶、罐、胶带等物品名称
    if (dataString.find("框") != std::string::npos) {
        grasp_msg.id = 2;
        grasp_msg.flag = true;
        grasp_msg.type = "box";
        text = "开始抓取框";
    } else if (dataString.find("瓶") != std::string::npos) {
        grasp_msg.id = 2;
        grasp_msg.flag = true;
        grasp_msg.type = "bottle";
        text = "开始抓取瓶子";
    } else if (dataString.find("罐") != std::string::npos) {
        grasp_msg.id = 2;
        grasp_msg.flag = true;
        grasp_msg.type = "can";
        text = "开始抓取罐";
    } else if (dataString.find("胶带") != std::string::npos) {
        grasp_msg.id = 2;
        grasp_msg.flag = true;
        grasp_msg.type = "sellotape";
        text = "开始抓取胶带";
    } else {
        ROS_WARN("Unrecognized object type");
        text = "未识别的物品类型";
    }
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: grasp %s", grasp_msg.type.c_str());
}

// 处理同步模式指令
void handleSyncMode(const std::string& dataString) {
    const char* text;
    in_sync_mode = true;
    text = "3秒后开始同步模式";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: after 3s start sync mode");
    std::this_thread::sleep_for(std::chrono::seconds(3));//3�������ͬ��ģʽ
    text = "开始同步";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: start sync mode");
}

// 处理退出同步模式指令
void handleExitSync(const std::string& dataString) {
    const char* text;
    in_sync_mode = false;
    text = "退出同步模式";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: exit sync mode");
}

// 处理放置初始姿态
void handlePlacingPose(const std::string& dataString) {
    const char* text;
    text = "设置为放置初始姿态";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: set to placing pose");
}

// 处理放置最终姿态
void handlePlacingFinalPose(const std::string& dataString) {
    const char* text;
    text = "设置为放置最终姿态";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: set to placing final pose");
}

// 处理夹爪闭合指令
void handleGripperClose(const std::string& dataString) {
    const char* text;
    text = "夹爪闭合";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper close");
}

// 处理夹爪张开指令
void handleGripperOpen(const std::string& dataString) {
    const char* text;
    text = "夹爪张开";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper open");
}

// 处理夹爪激活指令
void handleGripperActive(const std::string& dataString) {
    const char* text;
    text = "激活夹爪";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper active");
}

// 处理跟随物体
void handleFollowingObject(const std::string& dataString) {
    const char* text;
    // 判断dataString是否包含瓶子、盒子、罐头、胶带等物品
    if (dataString.find("瓶") != std::string::npos) {
        following_msg.type = "bottle";
        following_msg.flag = true;
        text = "开始跟随瓶子";
    } else if (dataString.find("盒") != std::string::npos) {
        following_msg.type = "box";
        following_msg.flag = true;
        text = "开始跟随盒子";
    } else if (dataString.find("罐") != std::string::npos) {
        following_msg.type = "can";
        following_msg.flag = true;
        text = "开始跟随罐头";
    } else if (dataString.find("胶带") != std::string::npos) {
        following_msg.type = "sellotape";
        following_msg.flag = true;
        text = "开始跟随胶带";
    } else {
        ROS_WARN("Unrecognized object type");
        text = "未识别物品类型";
    }
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: following %s", following_msg.type.c_str());
}

// 退出跟随物体
void handleExitFollowingObject(const std::string& dataString) {
    const char* text;
    following_msg.flag = false;
    text = "退出跟随模式";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: exit following object");
}

// 处理放置物体
void handlePlacingObject(const std::string& dataString) {
    const char* text;
    //判断dataString是否包含蓝色物品、绿色物品、粉色物品
    if (dataString.find("蓝") != std::string::npos) {
        placing_msg.id = 1;
        placing_msg.color = "blue";
        placing_msg.flag = true;
        text = "放到蓝色框中";
    } else if (dataString.find("绿") != std::string::npos) {
        placing_msg.id = 2;
        placing_msg.color = "green";
        placing_msg.flag = true;
        text = "放到绿色框中";
    } else if (dataString.find("粉") != std::string::npos) {
        placing_msg.id = 3;
        placing_msg.color = "pink";
        placing_msg.flag = true;
        text = "放到粉色框中";
    } else {
        placing_msg.id = 0;
        placing_msg.flag = true;
        ROS_WARN("Unrecognized object color");
        text = "未识别到物品";
    }
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: placing object to %s basket", placing_msg.color.c_str());
}

//夹爪控制函数
void gripperControl(double width)
{
    // 创建一个节点句柄
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    // 创建一个夹爪控制命令
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;
    // 打开夹爪
    command.rACT = 1;
    command.rGTO = 1;
    command.rSP = 255;
    command.rFR = 150;
    gripper_pub.publish(command);
    ros::Duration(0.1).sleep();  // 等待一段时间确保命令被执行
    // 设置夹爪的宽度
    command.rPR = static_cast<int>(width * 255);
    gripper_pub.publish(command);
    // 等待一段时间确保命令被执行
    ros::Duration(0.1).sleep();
    ROS_INFO("Gripper width set to %.2f", width);
}

//激活夹爪函数
void activeGripper()
{
    // 创建一个节点句柄
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    // 创建一个夹爪控制命令
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;
    // 关闭夹爪
    command.rACT = 0;
    gripper_pub.publish(command);
    ros::Duration(1.1).sleep();  // 等待一段时间确保命令被执行
    // 打开夹爪
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output activeGripperCommand;
    activeGripperCommand.rACT = 1;
    activeGripperCommand.rGTO = 1;
    activeGripperCommand.rSP = 255;
    activeGripperCommand.rFR = 150;
    gripper_pub.publish(activeGripperCommand);
    ROS_INFO("Gripper activated");
    ros::Duration(0.1).sleep();  // 等待一段时间确保命令被执行
}

// 线程循环夹爪控制函数
void gripperControlThread(double width) {
    // 当夹爪线程标志为true时，每隔2000ms执行一次夹爪控制
    while (gripperThreadRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        gripperControl(width);
    }
    // 退出线程
    ROS_INFO("Gripper control thread exited");
}

int main(int argc, char* argv[])
{
    //初始化ROS节点
    ros::init(argc,argv,"e5_voice_control");
    ros::NodeHandle n;
    //创建异步消息处理器
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //设置PLANNING_GROUP joint_model_group
    static const std::string PLANNING_GROUP = "manipulator_e5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //设置最大速度和最大加速度
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    //初始化同步模式标志位、夹爪线程运行标志位
    ros::Subscriber arm_synchronization_sub = n.subscribe("arm_synchronization", 1000, armcallback);
    ros::Subscriber iat_text_sub =n.subscribe("iat_voice_cut", 1000,iattextCallback); //subscribe voice to text reault
    tts_text_pub = n.advertise<std_msgs::String>("tts_text", 1000);  //publish text to voice string
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);//发布底盘控制指令
    //初始化抓取、跟随、放置消息发布器
    grasp_pub = n.advertise<aubo_grasp::graspMessage>("grasp", 10);//抓取物品发布器
    following_pub = n.advertise<aubo_grasp::followingMessage>("followingObject", 10);//跟随物品发布器
    placing_pub = n.advertise<aubo_grasp::placingMessage>("placingObject", 10);//放置物品发布器
    //获取当前关节状态
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    //初始化夹爪
    activeGripper();
    //等待命令
    ROS_INFO("Wait Command...");
    ros::Rate loop_rate(50);
    while(ros::ok()){
        if(!in_sync_mode){
        switch (cmd_type)
        {
        case 0:
            //退出
            goto exit;
            break;
        case 1:
            // 修改关节角度 (关节x旋转y)
            joint_group_positions[joint_num - 1] += rotation_angle * M_PI / 180.0; // radians
            move_group.setJointValueTarget(joint_group_positions);
            // 执行运动规划
            move_group.move();
            cmd_type = -1;
            break;
        case 2:
            //回到初始位置
            joint_group_positions[0] = 0;
            joint_group_positions[1] = 0;
            joint_group_positions[2] = 90 * M_PI / 180.0;
            joint_group_positions[3] = 0;
            joint_group_positions[4] = 90 * M_PI / 180.0;
            joint_group_positions[5] = 0;
            move_group.setJointValueTarget(joint_group_positions);
            // 执行运动规划
            move_group.move();
            cmd_type = -1;
            break;
        case 3:
            //抓取物品 id=1
            grasp_msg.id = 1;
            grasp_msg.flag = true;
            grasp_msg.type = "";
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 4:
            //抓取物品
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 5:
            //同步模式
            in_sync_mode = true;
            //设置夹爪线程标志位为true
            //gripperThreadRunning = true;
            break;
        case 6:
            //退出同步模式
            in_sync_mode = false;
            //设置夹爪线程标志位为false，并通知线程停止执行
            //gripperThreadRunning = false;
            gripperControl(0);
            cmd_type = 2;
            break;
        case 7:
            //移动到指定位置
            joint_group_positions[0] = 40 * M_PI / 180.0;
            joint_group_positions[1] = 0;
            joint_group_positions[2] = 60 * M_PI / 180.0;
            joint_group_positions[3] = -30 * M_PI / 180.0;
            joint_group_positions[4] = 90 * M_PI / 180.0;
            joint_group_positions[5] = -50 * M_PI / 180.0;
            move_group.setJointValueTarget(joint_group_positions);
            // 执行运动规划
            move_group.move();
            cmd_type = -1;
            break;
        case 8:
            //移动到指定位置
            joint_group_positions[0] = 40 * M_PI / 180.0;
            joint_group_positions[1] = -12 * M_PI / 180.0;
            joint_group_positions[2] = 80 * M_PI / 180.0;
            joint_group_positions[3] = 2 * M_PI / 180.0;
            joint_group_positions[4] = 90 * M_PI / 180.0;
            joint_group_positions[5] = -50 * M_PI / 180.0;
            move_group.setJointValueTarget(joint_group_positions);
            // 执行运动规划
            move_group.move();
            cmd_type = -1;
            break;
        case 11:
            //打开夹爪
            gripperControl(1);
            cmd_type = -1;
            break;
        case 12:
            //关闭夹爪
            gripperControl(0);
            cmd_type = -1;
            break;
        case 13:
            //激活夹爪
            activeGripper();
            cmd_type = -1;
            break;
        case 14:
            //跟随物品
            following_pub.publish(following_msg);
            cmd_type = -1;
            break;
        case 15:
            //放置物品
            placing_pub.publish(placing_msg);
            cmd_type = -1;
            break;
        case 16:
            //退出放置物品
            following_pub.publish(following_msg);
            cmd_type = -1;
            break;
        default:
            break;
        }
        }else{
                //设置关节4的角度为angle4
                joint_group_positions[3] = angle4 * M_PI / 180.0;
                //设置关节3的角度为angle3
                joint_group_positions[2] = angle3 * M_PI / 180.0;
                // 创建一个线程执行夹爪控制函数，并将夹爪宽度作为参数传入
                //std::thread gripperThread(gripperControlThread, gripper_width);
                //gripperThread.detach(); 
                //设置夹爪宽度
                gripperControl(gripper_width);
                //执行运动规划
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
                syncModeActive = true;  // 设置标志位为true执行
        }
        cmd_vel_pub.publish(twist);
        //ros::spinOnce();
        loop_rate.sleep();
    }
exit:
    spinner.stop();  // 停止 spinner
    ros::shutdown();
    return 0;
}