// handler.h

#include "oatpp/web/server/api/ApiController.hpp"
#include "oatpp/core/macro/codegen.hpp"
#include "oatpp/core/macro/component.hpp"

#include OATPP_CODEGEN_BEGIN(ApiController) // Begin Codegen

class ChatController : public oatpp::web::server::api::ApiController {
public:
  ChatController(OATPP_COMPONENT(std::shared_ptr<ObjectMapper>, objectMapper))
    : oatpp::web::server::api::ApiController(objectMapper) {}
  
  ENDPOINT("POST", "/chat/startchat", startChat, 
           BODY_STRING(String, commandStr)) {
             // 在此处添加处理语音识别的代码，并返回识别结果的字符串
    auto replyStr = "Received command: " + commandStr + ". This is a placeholder response.";
    return createResponse(Status::CODE_200, replyStr);
  }
  
};

#include OATPP_CODEGEN_END(ApiController) // End Codegen