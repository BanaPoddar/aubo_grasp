#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>

std::vector<std::string> commands; // 指令集合
ros::Publisher iat_voice_cut; // 发布语音指令切割

// 分割函数，使用多个关键词分割字符串
std::vector<std::string> split(const std::string &s, const std::vector<std::string> &keywords) {
    std::vector<std::string> result;
    size_t lastPos = 0; // 使用size_t而不是std::string::size_t
    for (const auto &keyword : keywords) {
        size_t pos = s.find(keyword, lastPos);
        if (pos != std::string::npos) {
            // 将找到的子字符串添加到结果中
            if (pos > lastPos) {
                result.push_back(s.substr(lastPos, pos - lastPos));
            }
            lastPos = pos + keyword.length();
        }
    }
    // 添加最后一个分割后的子字符串
    if (lastPos < s.length()) {
        result.push_back(s.substr(lastPos));
    }
    return result;
}

// 判断是否包含关键词
bool containsKeyword(const std::string& command) {
    std::vector<std::string> keywords = {"旋转","姿态","抓","初始","放","再见","同步","爪","退出","张开","关闭","激活","跟踪","跟随"};
    for (const auto& keyword : keywords) {
        if (command.find(keyword) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// 语音识别回调函数
void iattextCallback(const std_msgs::String::ConstPtr& msg) {
    std::string dataString = msg->data;
    std::cout << "Current dataString: " << dataString << std::endl;
    std::vector<std::string> keywords = {"，", "然后", "接着", "最后"};
    commands = split(dataString, keywords);
    for (const auto& command : commands) {
        if(command.empty()){
            continue;
        }
        std_msgs::String command_msg;
        command_msg.data = command;
        if (containsKeyword(command)) {
            iat_voice_cut.publish(command_msg);
            std::cout << "Current command: " << command << std::endl;
        } else {
            // 调用 chatgpt 模型
            std::cout << "Calling chatgpt for: " << command << std::endl;
            continue;
        }
        //如果不是最后一个指令，延时2秒
        if (command != commands.back()) {
        ros::Duration(2).sleep();
        }
    }
    std::cout << "——————————————————" << std::endl;
}

// chatgpt回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    std::string dataString = msg->data;
    std::cout << dataString << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "iat_voice_cut");
    ros::NodeHandle n;
    iat_voice_cut = n.advertise<std_msgs::String>("/iat_voice_cut", 1000);
    ros::Subscriber iat_text =n.subscribe("iat_text",1000,iattextCallback);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}