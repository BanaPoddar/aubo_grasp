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

//ע�⣺����ʶ��������ϳɵ����ı���Ϊgb2312
geometry_msgs::Twist twist;//���ӻ��ڵ㻰������
ros::Publisher cmd_vel_pub;//���ӻ��ڵ㻰�ⷢ����
std_msgs::String string_msg;//�����ϳɻ�������
ros::Publisher tts_text_pub;//�����ϳɻ��ⷢ����

//����ץȡ�����桢������Ʒ�������ݺͷ�����
aubo_grasp::graspMessage grasp_msg;// ץȡ���ⷢ����Ϣ
ros::Publisher grasp_pub;//ץȡ���ⷢ����
aubo_grasp::followingMessage following_msg;// ������Ʒ���ⷢ����Ϣ
ros::Publisher following_pub;//������Ʒ���ⷢ����
aubo_grasp::placingMessage placing_msg;// ������Ʒ���ⷢ����Ϣ
ros::Publisher placing_pub;//������Ʒ���ⷢ����

//����ȫ�ֱ���
int cmd_type = -1;//���ƻ�е��״����
int joint_num = -1;//�ؽں�
double angle3 = 90.0;//�ؽ�3�Ƕ�
double angle4 = 0.0;//�ؽ�4�Ƕ�
double gripper_width = 0.0;//��צ���
double rotation_angle = 0.0;//��ת�Ƕ�
bool in_sync_mode = false;//ͬ��ģʽ��־
bool syncModeActive = false;//����ͬ��ģʽ״̬�仯
std::atomic<bool> gripperThreadRunning(true); // ����һ��ԭ�Ӳ���ֵ����ʾ��צ�����߳��Ƿ�����

// ������������
enum CommandType {
    EXIT = 0,//�˳�
    JOINT_ROTATE = 1,//�ؽ���ת
    INITIAL_POSE = 2,//��ʼ��̬
    DIRECT_GRASP = 3,//���ץȡ
    GRASP_ITEM = 4,//ץȡ��Ʒ(���ӡ�ƿ�ӡ����ӡ�����)
    SYNC_MODE = 5,//ͬ��ģʽ
    EXIT_SYNC = 6,//�˳�ͬ��
    PLACING_POSE = 7,//���ó�ʼ��̬
    PLACING_FINAL_POSE = 8,//����������̬
    GRIPPER_CLOSE = 11,//��צ�պ�
    GRIPPER_OPEN = 12,//��צ��
    GRIPPER_ACTIVE = 13,//�����צ
    FOLLOWING_OBJECT = 14,//������Ʒ
    PLACING_OBJECT = 15,//������Ʒ
    EXIT_FOLLOWING = 16, //�˳�������Ʒ
};

// ���崦��������
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
void hamndlePlacingFinalPose(const std::string& dataString);

// �洢�ؼ��ʺͶ�Ӧ�Ĵ������Լ���������
std::map<std::string, std::pair<CommandHandler, CommandType>> commandMap = {
    {"�ټ�", {handleExit, EXIT}},
    {"��ת", {handleJointRotate, JOINT_ROTATE}},
    {"��ʼ", {handleInitialPose, INITIAL_POSE}},
    {"��ʼ��̬", {handleInitialPose, INITIAL_POSE}},
    {"ֱ��ץ", {handleDirectGrasp, DIRECT_GRASP}},
    {"���ץ", {handleDirectGrasp, DIRECT_GRASP}},
    {"ץȡ", {handleGraspItem, GRASP_ITEM}},
    {"ץ��", {handleGraspItem, GRASP_ITEM}},
    {"ͬ��ģʽ", {handleSyncMode, SYNC_MODE}},
    {"ͬ����̬", {handleSyncMode, SYNC_MODE}},
    {"�˳�ͬ��", {handleExitSync, EXIT_SYNC}},
    {"������̬", {handlePlacingPose, PLACING_POSE}},
    {"������̬", {hamndlePlacingFinalPose, PLACING_FINAL_POSE}},
    {"�ر�צ", {handleGripperClose, GRIPPER_CLOSE}},
    {"�պϼ�צ", {handleGripperClose, GRIPPER_CLOSE}},
    {"�ſ�צ", {handleGripperOpen, GRIPPER_OPEN}},
    {"�ſ�ת", {handleGripperOpen, GRIPPER_OPEN}},
    {"�ſ���צ", {handleGripperOpen, GRIPPER_OPEN}},
    {"�����צ", {handleGripperActive, GRIPPER_ACTIVE}},
    {"�����״", {handleGripperActive, GRIPPER_ACTIVE}},
    {"���צ", {handleGripperActive, GRIPPER_ACTIVE}},
    {"����", {handleFollowingObject, FOLLOWING_OBJECT}},
    {"����", {handleFollowingObject, FOLLOWING_OBJECT}},
    {"�˳���", {handleExitFollowingObject, EXIT_FOLLOWING}},
    {"��", {handlePlacingObject, PLACING_OBJECT}},
};

//��������ӳ��
std::map<std::string, int> chineseNumberMap = {
    {"һ", 1},{"��", 1},{"Ҫ", 1},{"��", 1}, {"��", 1},{"��", 2}, {"��", 3}, {"��", 4}, {"��", 4}, {"ʽ", 4},
    {"��", 5}, {"��", 5}, {"��", 6}, {"��", 7}, {"��", 8}, {"��", 9}, {"ʮ", 10},
    {"ʮһ", 11}, {"ʮ��", 12}, {"ʮ��", 13}, {"ʮ��", 14}, {"ʮ��", 15}, {"ʮ��", 16},
    {"ʮ��", 17}, {"ʮ��", 18}, {"ʮ��", 19}, {"��ʮ", 20}
};

//ͬ��ģʽ�ص�����
void armcallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 5) {
        //��ӡ4������
        //ROS_INFO("Received data: %f", msg->data[2]);
        double angle1 = msg->data[0];
        //����ؽ�4�ĽǶ�С��-40������Ϊ-40���������90������Ϊ90
        if(angle1 < 50) angle4 = -40;
        else if(angle1 > 180) angle4 = 90;
        else angle4 = angle1 -90;
        //���ùؽ�3�ĽǶ�Ϊ180 - angle2 ����ؽ�3�ĽǶ�С��30������Ϊ30���������115������Ϊ115
        double angle2 = msg->data[1];
        if(angle2 < 65) angle3 = 115;
        else if(angle2 > 150) angle3 = 30;
        else angle3 = 180 - angle2;
        //ROS_INFO("Received angle4: %f, angle3: %f", angle4, angle3);
        //���thumb_index_angleС��6������Ϊ6���������45������Ϊ45
        double thumb_index_data = msg->data[2];
        if(thumb_index_data < 6) gripper_width = 1.0;
        else if(thumb_index_data > 45) gripper_width = 0.0;
        else gripper_width = 1.0 - (thumb_index_data - 6) / 39.0;
        //ROS_INFO("Received data: %f", gripper_width);
        //���ܱ�־λ ���msg->data[4]Ϊ1������Ϊtrue����������Ϊfalse
        in_sync_mode = msg->data[4] == 1;
        // �����1��Ϊ0��ִ��һ�η��س�ʼ��̬�Ĳ���
        if (msg->data[4] == 0 && syncModeActive) {
            // ִ��һ�η��س�ʼ��̬�Ĳ���
            cmd_type = 6;
            syncModeActive = false;  // ���ñ�־λΪ��ִ��
        }
    } else {
        ROS_WARN("Received incomplete data.");
    }
}

// ����ʶ��ص�����
void iattextCallback(const std_msgs::String::ConstPtr& msg) {
    const char* text;
    std::string dataString = msg->data;
    //ָ��ʶ��
    for (const auto& pair : commandMap) {
        if (dataString.find(pair.first) != std::string::npos) {
            pair.second.first(dataString);
            //ROS_INFO("Received command: %s", pair.first.c_str());
            cmd_type = pair.second.second;
            return;
        }
    }
    //δʶ������������ϳɷ���
    ROS_WARN("Unrecognized command");
    cmd_type = -1;
    text = "δʶ�������";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
}

//��ȡ�ؽں�
int getJointNumber(const std::string& str) {
    for (const auto& pair : chineseNumberMap) {
        if (str.find(pair.first) != std::string::npos) {
            //���pair.first����1С��6�򷵻�-1�����򷵻�pair.second
            if (pair.second > 6 || pair.second < 1) return -1;// δʶ��Ĺؽں�
            else return pair.second;
        }
    }
    return -1;  // δʶ��Ĺؽں�
}

//��ȡ��ת�Ƕ�
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

// �����˳�����
void handleExit(const std::string& dataString) {
    const char* text;
    text = "���˳���������";
}

// ����ؽ���ת����
void handleJointRotate(const std::string& dataString) {
    const char* text;
    //��ӡ����ʶ����
    std::string joint_num_str;
    std::string rotation_angle_str;
    size_t pos = dataString.find("��ת");
    //����joint_num_str�ؽڲ��� rotation_angle_str��ת�ǶȲ���
    joint_num_str = dataString.substr(0, pos);
    rotation_angle_str = dataString.substr(pos + 4);//gb2312�����ʽ��"��ת"�ĳ���Ϊ4��utf8Ϊ6��
    //��ȡ�ؽںź���ת�Ƕ�
    joint_num = getJointNumber(joint_num_str);
    rotation_angle = getRotationAngle(rotation_angle_str);
    if (joint_num != -1 && rotation_angle != 0.0) {
        //����̨����ؽ�%d��ת%.1f��
        ROS_INFO("Received command: joint %d rotate %.1f degrees", joint_num, rotation_angle);
        //ƴ�����������������ϳɻ���
        char forwordString[40];
        snprintf(forwordString, sizeof(forwordString), "�ؽ�%d��ת%.1f��", joint_num, rotation_angle);
        text = forwordString;
        string_msg.data = text;
        tts_text_pub.publish(string_msg);
    } else {
        ROS_WARN("Unrecognized joint number or rotation angle");
        text = "δʶ�𵽹ؽںŻ���ת�Ƕ�";  
        string_msg.data = text;
        tts_text_pub.publish(string_msg);      
    }
}

// �����ʼ��̬����
void handleInitialPose(const std::string& dataString) {
    const char* text;
    text = "����Ϊ��ʼ��̬";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: set to initial pose");
}

// �������/ֱ��ץȡ����
void handleDirectGrasp(const std::string& dataString) {
    const char* text;
    text = "��ʼ���ץȡ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: direct grasp");
}

// ����ץȡ��Ʒ����
void handleGraspItem(const std::string& dataString) {
    const char* text;
    //�ж�dataString���Ƿ��С��С���ƿ�����ޡ���������
    if (dataString.find("��") != std::string::npos) {
        grasp_msg.id = 2;
        grasp_msg.flag = true;
        grasp_msg.type = "box";
        text = "��ʼץȡ����";
    } else if (dataString.find("ƿ") != std::string::npos) {
        grasp_msg.id = 2;
        grasp_msg.flag = true;
        grasp_msg.type = "bottle";
        text = "��ʼץȡƿ��";
    } else if (dataString.find("��") != std::string::npos) {
        grasp_msg.id = 2;
        grasp_msg.flag = true;
        grasp_msg.type = "can";
        text = "��ʼץȡ����";
    } else if (dataString.find("����") != std::string::npos) {
        grasp_msg.id = 2;
        grasp_msg.flag = true;
        grasp_msg.type = "sellotape";
        text = "��ʼץȡ����";
    } else {
        ROS_WARN("Unrecognized object type");
        text = "δʶ����Ʒ����";
    }
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: grasp %s", grasp_msg.type.c_str());
}

// ����ͬ��ģʽ����
void handleSyncMode(const std::string& dataString) {
    const char* text;
    in_sync_mode = true;
    text = "3�������ͬ��ģʽ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: after 3s start sync mode");
    std::this_thread::sleep_for(std::chrono::seconds(3));//3�������ͬ��ģʽ
    text = "��ʼͬ��";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: start sync mode");
}

// �����˳�ͬ������
void handleExitSync(const std::string& dataString) {
    const char* text;
    in_sync_mode = false;
    text = "�˳�ͬ��ģʽ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: exit sync mode");
}

// ���������̬����
void handlePlacingPose(const std::string& dataString) {
    const char* text;
    text = "����Ϊ������̬";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: set to placing pose");
}

// �������������̬����
void hamndlePlacingFinalPose(const std::string& dataString) {
    const char* text;
    text = "����Ϊ����������̬";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: set to placing final pose");
}

// �����צ�պ�����
void handleGripperClose(const std::string& dataString) {
    const char* text;
    text = "��צ�պ�";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper close");
}

// �����צ������
void handleGripperOpen(const std::string& dataString) {
    const char* text;
    text = "��צ��";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper open");
}

// �������צ����
void handleGripperActive(const std::string& dataString) {
    const char* text;
    text = "�����צ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: gripper active");
}

// ���������Ʒ����
void handleFollowingObject(const std::string& dataString) {
    const char* text;
    //�ж�dataString���Ƿ��С�ƿ�����С����ޡ���������
    if (dataString.find("ƿ") != std::string::npos) {
        following_msg.type = "bottle";
        following_msg.flag = true;
        text = "��ʼ����ƿ��";
    } else if (dataString.find("��") != std::string::npos) {
        following_msg.type = "box";
        following_msg.flag = true;
        text = "��ʼ�������";
    } else if (dataString.find("��") != std::string::npos) {
        following_msg.type = "can";
        following_msg.flag = true;
        text = "��ʼ�������";
    } else if (dataString.find("����") != std::string::npos) {
        following_msg.type = "sellotape";
        following_msg.flag = true;
        text = "��ʼ���潺��";
    } else {
        ROS_WARN("Unrecognized object type");
        text = "δʶ����Ʒ����";
    }
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: following %s", following_msg.type.c_str());
}

// �����˳�������Ʒ����
void handleExitFollowingObject(const std::string& dataString) {
    const char* text;
    following_msg.flag = false;
    text = "�˳�����ģʽ";
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: exit following object");
}

// ���������Ʒ����
void handlePlacingObject(const std::string& dataString) {
    const char* text;
    //�ж�dataString���Ƿ��С��������̡����ۡ�
    if (dataString.find("��") != std::string::npos) {
        placing_msg.id = 1;
        placing_msg.color = "blue";
        placing_msg.flag = true;
        text = "�ŵ���ɫ����";
    } else if (dataString.find("��") != std::string::npos) {
        placing_msg.id = 2;
        placing_msg.color = "green";
        placing_msg.flag = true;
        text = "�ŵ���ɫ����";
    } else if (dataString.find("��") != std::string::npos) {
        placing_msg.id = 3;
        placing_msg.color = "pink";
        placing_msg.flag = true;
        text = "�ŵ���ɫ����";
    } else {
        placing_msg.id = 0;
        placing_msg.flag = true;
        ROS_WARN("Unrecognized object color");
        text = "δʶ�𵽷��õ�";
    }
    string_msg.data = text;
    tts_text_pub.publish(string_msg);
    ROS_INFO("Received command: placing object to %s basket", placing_msg.color.c_str());
}

//��צ���ƺ���������width
void gripperControl(double width)
{
    // ����һ��������
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    // ����һ����צ��������
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;
    // �����צ
    command.rACT = 1;
    command.rGTO = 1;
    command.rSP = 255;
    command.rFR = 150;
    gripper_pub.publish(command);
    ros::Duration(0.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
    // ���ü�צ�Ŀ��
    command.rPR = static_cast<int>(width * 255);
    gripper_pub.publish(command);
    // �ȴ�����ȷ���������
    ros::Duration(0.1).sleep();
    ROS_INFO("Gripper width set to %.2f", width);
}

//�����צ����
void activeGripper()
{
    // ����һ��������
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    // ����һ����צ��������
     robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;
    // ���ü�צ
     command.rACT = 0;
     gripper_pub.publish(command);
     ros::Duration(1.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
    // �����צ
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output activeGripperCommand;
    activeGripperCommand.rACT = 1;
    activeGripperCommand.rGTO = 1;
    activeGripperCommand.rSP = 255;
    activeGripperCommand.rFR = 150;
    gripper_pub.publish(activeGripperCommand);
    ROS_INFO("Gripper activated");
    ros::Duration(0.1).sleep();  // �ȴ�һ��ʱ����ȷ�����ִ��
}

// �̼߳�צ���ƺ���
void gripperControlThread(double width) {
    // ����צ�����̱߳�־Ϊtrueʱ��ÿ��2000msִ��һ�μ�צ����
    while (gripperThreadRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        gripperControl(width);
    }
    // �˳��߳�
    ROS_INFO("Gripper control thread exited");
}

int main(int argc, char* argv[])
{
    //��ʼ��ROS�ڵ�
    ros::init(argc,argv,"e5_voice_control");
    ros::NodeHandle n;
    //�����첽��Ϣ����
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //����PLANNING_GROUP joint_model_group
    static const std::string PLANNING_GROUP = "manipulator_e5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //��������ٶȺͼ��ٶ�
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    //��ʼ��ͬ��ģʽ������ʶ��������ϳɡ����ӻ��ڵ㻰��
    ros::Subscriber arm_synchronization_sub = n.subscribe("arm_synchronization", 1000, armcallback);
    ros::Subscriber iat_text_sub =n.subscribe("iat_voice_cut", 1000,iattextCallback); //subscribe voice to text reault
    tts_text_pub = n.advertise<std_msgs::String>("tts_text", 1000);  //publish text to voice string
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);//���ӻ��ڵ㻰�ⷢ��
    //��ʼ��ץȡ�����桢������Ʒ����
    grasp_pub = n.advertise<aubo_grasp::graspMessage>("grasp", 10);//ץȡ���ⷢ����
    following_pub = n.advertise<aubo_grasp::followingMessage>("followingObject", 10);//������Ʒ���ⷢ����
    placing_pub = n.advertise<aubo_grasp::placingMessage>("placingObject", 10);//������Ʒ���ⷢ����
    //��ȡ��ǰ��е�۵�״̬
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    //��ʼ����צ
    activeGripper();
    //�ȴ���е�۳�ʼ��
    ROS_INFO("Wait Command...");
    ros::Rate loop_rate(50);
    while(ros::ok()){
        if(!in_sync_mode){
        switch (cmd_type)
        {
        case 0:
            //�˳�
            goto exit;
            break;
        case 1:
            // �ı�ؽڽǶ� (���ؽ�x��תy��)
            joint_group_positions[joint_num - 1] += rotation_angle * M_PI / 180.0; // radians
            move_group.setJointValueTarget(joint_group_positions);
            // �滮��ִ���˶�
            move_group.move();
            cmd_type = -1;
            break;
        case 2:
            //���û�е��homeλ��
            joint_group_positions[0] = 0;
            joint_group_positions[1] = 0;
            joint_group_positions[2] = 90 * M_PI / 180.0;
            joint_group_positions[3] = 0;
            joint_group_positions[4] = 90 * M_PI / 180.0;
            joint_group_positions[5] = 0;
            move_group.setJointValueTarget(joint_group_positions);
            // �滮��ִ���˶�
            move_group.move();
            cmd_type = -1;
            break;
        case 3:
            //���û�е��ֱ��ץȡ ����ץȡ���� id=1
            grasp_msg.id = 1;
            grasp_msg.flag = true;
            grasp_msg.type = "";
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 4:
            //���û�е��ץȡ��Ʒ ����ץȡ����
            grasp_pub.publish(grasp_msg);
            cmd_type = -1;
            break;
        case 5:
            //ͬ��ģʽ
            in_sync_mode = true;
            //���ü�צ�����̱߳�־Ϊtrue
            //gripperThreadRunning = true;
            break;
        case 6:
            //�˳�ͬ��ģʽ
            in_sync_mode = false;
            //���ü�צ�����̱߳�־Ϊfalse��֪ͨ�߳�ִֹͣ��
            //gripperThreadRunning = false;
            gripperControl(0);
            cmd_type = 2;
            break;
        case 7:
            //���û�е�۷���λ��
            joint_group_positions[0] = 40 * M_PI / 180.0;
            joint_group_positions[1] = 0;
            joint_group_positions[2] = 60 * M_PI / 180.0;
            joint_group_positions[3] = -30 * M_PI / 180.0;
            joint_group_positions[4] = 90 * M_PI / 180.0;
            joint_group_positions[5] = -50 * M_PI / 180.0;
            move_group.setJointValueTarget(joint_group_positions);
            // �滮��ִ���˶�
            move_group.move();
            cmd_type = -1;
            break;
        case 8:
            //���û�е�۷�������λ��
            joint_group_positions[0] = 40 * M_PI / 180.0;
            joint_group_positions[1] = -12 * M_PI / 180.0;
            joint_group_positions[2] = 80 * M_PI / 180.0;
            joint_group_positions[3] = 2 * M_PI / 180.0;
            joint_group_positions[4] = 90 * M_PI / 180.0;
            joint_group_positions[5] = -50 * M_PI / 180.0;
            move_group.setJointValueTarget(joint_group_positions);
            // �滮��ִ���˶�
            move_group.move();
            cmd_type = -1;
            break;
        case 11:
            //�պϼ�צ
            gripperControl(1);
            cmd_type = -1;
            break;
        case 12:
            //�ſ���צ
            gripperControl(0);
            cmd_type = -1;
            break;
        case 13:
            //�����צ
            activeGripper();
            cmd_type = -1;
            break;
        case 14:
            //���û�е�۸���
            following_pub.publish(following_msg);
            cmd_type = -1;
            break;
        case 15:
            //���û�е�۷�����Ʒ �������û���
            placing_pub.publish(placing_msg);
            cmd_type = -1;
            break;
        case 16:
            //�˳�������Ʒ
            following_pub.publish(following_msg);
            cmd_type = -1;
            break;
        default:
            break;
        }
        }else{
                //���ùؽ�4�ĽǶ�Ϊangle4
                joint_group_positions[3] = angle4 * M_PI / 180.0;
                //���ùؽ�3�ĽǶ�Ϊangle3
                joint_group_positions[2] = angle3 * M_PI / 180.0;
                // ����һ�����߳�ִ�м�צ���ƣ�����������Ϊ��̨�߳�
                //std::thread gripperThread(gripperControlThread, gripper_width);
                //gripperThread.detach(); 
                //���ü�צ���
                gripperControl(gripper_width);
                //ִ�л�е���ƶ�
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
                syncModeActive = true;  // ���ñ�־λΪ��ִ��
        }
        cmd_vel_pub.publish(twist);
        //ros::spinOnce();
        loop_rate.sleep();
    }
exit:
    spinner.stop();  // ���ʱֹͣ spinner
    ros::shutdown();
    return 0;
}