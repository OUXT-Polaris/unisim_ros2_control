/**
 * unisim_ros2_control_api
 * No description provided (generated by Swagger Codegen https://github.com/swagger-api/swagger-codegen)
 *
 * OpenAPI spec version: 0.0.1
 * 
 *
 * NOTE: This class is auto generated by the swagger code generator 2.4.22.
 * https://github.com/swagger-api/swagger-codegen.git
 * Do not edit the class manually.
 */

/*
 * Posts.h
 *
 * 
 */

#ifndef IO_SWAGGER_CLIENT_MODEL_Posts_H_
#define IO_SWAGGER_CLIENT_MODEL_Posts_H_


#include "../ModelBase.h"

#include <cpprest/details/basic_types.h>

namespace io {
namespace swagger {
namespace client {
namespace model {

/// <summary>
/// 
/// </summary>
class  Posts
    : public ModelBase
{
public:
    Posts();
    virtual ~Posts();

    /////////////////////////////////////////////
    /// ModelBase overrides

    void validate() override;

    web::json::value toJson() const override;
    void fromJson(web::json::value& json) override;

    void toMultipart(std::shared_ptr<MultipartFormData> multipart, const utility::string_t& namePrefix) const override;
    void fromMultiPart(std::shared_ptr<MultipartFormData> multipart, const utility::string_t& namePrefix) override;

    /////////////////////////////////////////////
    /// Posts members

    /// <summary>
    /// 
    /// </summary>
    utility::string_t getName() const;
    bool nameIsSet() const;
    void unsetName();
    void setName(utility::string_t value);
    /// <summary>
    /// 
    /// </summary>
    utility::string_t getTitle() const;
    bool titleIsSet() const;
    void unsetTitle();
    void setTitle(utility::string_t value);
    /// <summary>
    /// 
    /// </summary>
    bool isPublished() const;
    bool publishedIsSet() const;
    void unsetPublished();
    void setPublished(bool value);
    /// <summary>
    /// 
    /// </summary>
    utility::string_t getContent() const;
    bool contentIsSet() const;
    void unsetContent();
    void setContent(utility::string_t value);

protected:
    utility::string_t m_Name;
    bool m_NameIsSet;
    utility::string_t m_Title;
    bool m_TitleIsSet;
    bool m_Published;
    bool m_PublishedIsSet;
    utility::string_t m_Content;
    bool m_ContentIsSet;
};

}
}
}
}

#endif /* IO_SWAGGER_CLIENT_MODEL_Posts_H_ */