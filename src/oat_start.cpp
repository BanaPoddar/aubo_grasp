#include "oatpp/web/server/HttpRequestHandler.hpp"
#include "oatpp/web/server/HttpConnectionHandler.hpp"
#include "oatpp/parser/json/mapping/Serializer.hpp"
#include "oatpp/network/tcp/server/ConnectionProvider.hpp"
#include "oatpp/network/Server.hpp"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iconv.h>
#include <cstring>
#include <iomanip>
#include <vector>

ros::Publisher oat_start_publisher;
std::vector<std::string> commands; // 指令集
std::string replyStr = "";
std::string utf8_command = "";

// 解码URL编码的字符串
std::string urlDecode(const std::string &urlEncodedStr) {
    std::ostringstream decodedStream;
    for (size_t i = 0; i < urlEncodedStr.length(); ++i) {
        if (urlEncodedStr[i] == '%') {
            if (i + 2 < urlEncodedStr.length()) {
                int hexValue;
                std::istringstream hexStream(urlEncodedStr.substr(i + 1, 2));
                if (hexStream >> std::hex >> hexValue) {
                    decodedStream << static_cast<char>(hexValue);
                    i += 2;
                } else {
                    // 处理无效的十六进制值
                    decodedStream << urlEncodedStr[i];
                }
            } else {
                // 处理不完整的百分号编码
                decodedStream << urlEncodedStr[i];
            }
        } else if (urlEncodedStr[i] == '+') {
            decodedStream << ' ';
        } else {
            decodedStream << urlEncodedStr[i];
        }
    }
    return decodedStream.str();
}

// 将UTF-8编码的字符串转换为GB2312编码
std::string utf8ToGb2312(const std::string &utf8Str) {
    iconv_t cd = iconv_open("gb2312", "utf-8");
    if (cd == (iconv_t)-1) {
        std::cerr << "Failed to open iconv" << std::endl;
        return "";
    }

    size_t inSize = utf8Str.size();
    size_t outSize = inSize * 2; // 预留足够的空间
    char *inBuf = const_cast<char*>(utf8Str.c_str());
    char *outBuf = new char[outSize];
    char *outPtr = outBuf;

    if (iconv(cd, &inBuf, &inSize, &outPtr, &outSize) == (size_t)-1) {
        std::cerr << "Failed to convert" << std::endl;
        iconv_close(cd);
        delete[] outBuf;
        return "";
    }
    iconv_close(cd);
    std::string result(outBuf, outPtr - outBuf);
    delete[] outBuf;
    return result;
}

//将指令按照关键词分割
std::vector<std::string> split(const std::string& text, const std::string& delim) {
    std::vector<std::string> tokens;
    size_t start = 0, end = 0;
    const std::string keywords[] = {"然后", "最后", "接着"};
    while ((end = text.find(delim, start)) != std::string::npos) {
        std::string token = text.substr(start, end - start);
        if (!token.empty()) {
            tokens.push_back(token);
        }
        start = end + delim.length();
    }

    std::string last_token = text.substr(start);
    if (!last_token.empty()) {
        tokens.push_back(last_token); // 添加最后一个片段
    }
    // 检查关键词是否存在于每个片段中，如果存在则进行二次拆分
    std::vector<std::string> temp_tokens;
    for (const auto& token : tokens) {
        bool found = false;
        for (const auto& keyword : keywords) {
            size_t pos = token.find(keyword);
            if (pos != std::string::npos) {
                found = true;
                auto sub_tokens = split(token.substr(pos + keyword.length()), keyword);
                for (const auto& sub_token : sub_tokens) {
                    if (!sub_token.empty()) {
                        temp_tokens.push_back(sub_token);
                    }
                }
                if (!temp_tokens.empty()) {
                    temp_tokens.push_back(token.substr(0, pos));
                }
                break;
            }
        }
        if (!found) {
            temp_tokens.push_back(token);
        }
    }
    // 移除最后一个空字符串
    if (!temp_tokens.empty() && temp_tokens.back().empty()) {
        temp_tokens.pop_back();
    }
    return temp_tokens;
}

//设置返回值
void processCommand(const std::string& command) {
    if (command.find("旋转") != std::string::npos) {
        replyStr = "机械臂执行旋转命令";
    } else if (command.find("初始姿态") != std::string::npos) {
        replyStr = "机械臂设置为初始姿态";
    } else if (command.find("抓取") != std::string::npos) {
        replyStr = "机械臂执行抓取命令";
    } else if (command.find("同步模式") != std::string::npos) {
        replyStr = "即将进入同步模式";
    } else if (command.find("退出同步") != std::string::npos) {
        replyStr = "已退出同步模式";
    } else {
        replyStr = "未识别的指令";
    }
}

// 自定义请求处理程序 voiceControl
#define O_UNUSED(x) (void)x;
class VoiceControl : public oatpp::web::server::HttpRequestHandler
{
public:
    // 处理传入的请求，并返回响应
    std::shared_ptr<OutgoingResponse> handle(const std::shared_ptr<IncomingRequest>& request) override {
        O_UNUSED(request);
        // 获取请求参数 commandStr
        oatpp::String commandStr = request->getQueryParameters().get("commandStr");
        utf8_command = urlDecode(commandStr->c_str());
        //std::string gb2312_command =utf8ToGb2312(utf8_command);
        std::vector<std::string> replyArray;
        oatpp::List<oatpp::String> oatppStringList;
        commands = split(utf8_command, "，");
        for (const auto& command : commands) {
            if(command.empty()){
                continue;
            }
            processCommand(command);
            std::cout << "Current reply: " << replyStr << std::endl;
            replyArray.push_back(replyStr);
            //oatppStringList->push_back(oatpp::String(replyStr));
        }
        // 创建一个 ROS 消息对象
        std_msgs::String rosMsg;
        rosMsg.data = utf8ToGb2312(urlDecode(commandStr->c_str())); // 将 oatpp 的 String 转换为 std::string 并赋值给 ROS 消息对象
        // 发布 ROS 消息
        oat_start_publisher.publish(rosMsg);
        // 返回响应
        oatpp::parser::json::mapping::Serializer serializer;
        auto jsonStringStream = std::make_shared<oatpp::data::stream::BufferOutputStream>();
        serializer.serializeToStream(jsonStringStream.get(), oatppStringList);
        auto jsonString = jsonStringStream->toString();
        auto response = ResponseFactory::createResponse(Status::CODE_200, "1");
        response->putHeader(Header::CONTENT_TYPE, "application/json");
        return response;
    }
};

void oat_run()
{
    // 为 HTTP 请求创建路由器
    auto router = oatpp::web::server::HttpRouter::createShared();
    // 路由 GET - "/hello" 请求到处理程序
    // router->route("GET", "/hello", std::make_shared<test>());
    router->route("GET", "/chat/startchat", std::make_shared<VoiceControl>()); // 添加新的路由
    // 创建 HTTP 连接处理程序
    auto connectionHandler = oatpp::web::server::HttpConnectionHandler::createShared(router);
    // 创建 TCP 连接提供者
    auto connectionProvider = oatpp::network::tcp::server::ConnectionProvider::createShared({"0.0.0.0", 8080, oatpp::network::Address::IP_4});
    // 创建服务器，它接受提供的 TCP 连接并将其传递给 HTTP 连接处理程序
    oatpp::network::Server server(connectionProvider, connectionHandler);
    // 打印服务器端口
    OATPP_LOGI("MyApp", "Server running on port %s", connectionProvider->getProperty("port").getData());
    // 运行服务器
    server.run();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oat_start");
    ros::NodeHandle n;
    oat_start_publisher = n.advertise<std_msgs::String>("/iat_text", 1000);
    ros::Rate loop_rate(10);
    // 初始化 oatpp 环境
    oatpp::base::Environment::init();
    // 运行应用
    oat_run();
    // 进入 ROS 循环，处理 ROS 回调函数
    ros::spin();
    // 销毁 oatpp 环境
    oatpp::base::Environment::destroy();
    ros::shutdown();
    return 0;
}