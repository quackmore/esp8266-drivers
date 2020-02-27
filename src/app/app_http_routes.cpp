/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <quackmore-ff@yahoo.com> wrote this file.  As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy me a beer in return. Quackmore
 * ----------------------------------------------------------------------------
 */

// SDK includes
extern "C"
{
#include "osapi.h"
#include "user_interface.h"
#include "mem.h"
#include "ip_addr.h"
}

#include "app.hpp"
#include "app_http_routes.hpp"
#include "app_test.hpp"
#include "espbot.hpp"
#include "espbot_diagnostic.hpp"
#include "espbot_global.hpp"
#include "espbot_json.hpp"
#include "espbot_utils.hpp"
#include "espbot_webserver.hpp"
#include "library.hpp"

static void get_api_info(struct espconn *ptr_espconn, Http_parsed_req *parsed_req)
{
    ALL("get_api_info");
    int str_len = os_strlen(app_name) +
                  os_strlen(app_release) +
                  os_strlen(espbot.get_name()) +
                  os_strlen(espbot.get_version()) +
                  os_strlen(library_release) +
                  10 +
                  os_strlen(system_get_sdk_version()) +
                  10;
    Heap_chunk msg(155 + str_len, dont_free);
    if (msg.ref)
    {
        fs_sprintf(msg.ref,
                   "{\"app_name\":\"%s\",\"app_version\":\"%s\",\"espbot_name\":\"%s\",",
                   app_name,
                   app_release,
                   espbot.get_name());
        fs_sprintf((msg.ref + os_strlen(msg.ref)),
                   "\"espbot_version\":\"%s\",\"library_version\":\"%s\",\"chip_id\":\"%d\",",
                   espbot.get_version(),
                   library_release,
                   system_get_chip_id());
        fs_sprintf(msg.ref + os_strlen(msg.ref),
                   "\"sdk_version\":\"%s\",\"boot_version\":\"%d\"}",
                   system_get_sdk_version(),
                   system_get_boot_version());
        http_response(ptr_espconn, HTTP_OK, HTTP_CONTENT_JSON, msg.ref, true);
        // esp_free(msg); // dont't free the msg buffer cause it could not have been used yet
    }
    else
    {
        // esp_diag.error(APP_GET_API_INFO_HEAP_EXHAUSTED, 155 + str_len);
        ERROR("get_api_info heap exhausted %d", 155 + str_len);
    }
}

static void get_api_test(struct espconn *ptr_espconn, Http_parsed_req *parsed_req)
{
    ALL("get_api_test");
    int test_number;
    Json_str test_cfg(parsed_req->req_content, parsed_req->content_len);
    if (test_cfg.syntax_check() == JSON_SINTAX_OK)
    {
        if (test_cfg.find_pair(f_str("test_number")) != JSON_NEW_PAIR_FOUND)
        {
            http_response(ptr_espconn, HTTP_BAD_REQUEST, HTTP_CONTENT_JSON, f_str("Cannot find JSON string 'test_number'"), false);
            return;
        }
        if (test_cfg.get_cur_pair_value_type() != JSON_INTEGER)
        {
            http_response(ptr_espconn, HTTP_BAD_REQUEST, HTTP_CONTENT_JSON, f_str("JSON pair with string 'test_number' does not have a INTEGER value type"), false);
            return;
        }
        Heap_chunk tmp_test_number(test_cfg.get_cur_pair_value_len());
        if (tmp_test_number.ref == NULL)
        {
            // esp_diag.error(APP_GET_API_TEST_HEAP_EXHAUSTED, test_cfg.get_cur_pair_value_len());
            ERROR("get_api_test heap exhausted %d", test_cfg.get_cur_pair_value_len() + 1);
            http_response(ptr_espconn, HTTP_SERVER_ERROR, HTTP_CONTENT_JSON, f_str("not enough heap memory"), false);
            return;
        }
        os_strncpy(tmp_test_number.ref, test_cfg.get_cur_pair_value(), test_cfg.get_cur_pair_value_len());
        test_number = atoi(tmp_test_number.ref);
        espmem.stack_mon();
    }
    else
    {
        http_response(ptr_espconn, HTTP_BAD_REQUEST, HTTP_CONTENT_JSON, f_str("Json bad syntax"), false);
        return;
    }
    Heap_chunk msg(36, dont_free);
    if (msg.ref)
    {
        fs_sprintf(msg.ref, "{\"test_number\": %d}", test_number);
        http_response(ptr_espconn, HTTP_OK, HTTP_CONTENT_TEXT, msg.ref, true);
        // esp_free(msg); // dont't free the msg buffer cause it could not have been used yet
        run_test(test_number);
    }
    else
    {
        // esp_diag.error(APP_GET_API_TEST_HEAP_EXHAUSTED, 36);
        ERROR("get_api_test heap exhausted %d", 36);
    }
}

bool app_http_routes(struct espconn *ptr_espconn, Http_parsed_req *parsed_req)
{
    if ((0 == os_strcmp(parsed_req->url, f_str("/api/info"))) && (parsed_req->req_method == HTTP_GET))
    {
        get_api_info(ptr_espconn, parsed_req);
        return true;
    }
    if ((0 == os_strcmp(parsed_req->url, f_str("/api/test"))) && (parsed_req->req_method == HTTP_POST))
    {
        get_api_test(ptr_espconn, parsed_req);
        return true;
    }
    return false;
}