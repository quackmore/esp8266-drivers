function get_code_str(code) { var code_str = [];
code_str[parseInt("0010", 16)] = "FILE_TO_JSON_FS_NOT_AVAILABLE";
code_str[parseInt("0011", 16)] = "FILE_TO_JSON_FILE_NOT_FOUND";
code_str[parseInt("0012", 16)] = "FILE_TO_JSON_CANNOT_OPEN_FILE";
code_str[parseInt("0013", 16)] = "FILE_TO_JSON_HEAP_EXHAUSTED";
code_str[parseInt("0014", 16)] = "FILE_TO_JSON_CANNOT_PARSE_JSON";
code_str[parseInt("0015", 16)] = "FILE_TO_JSON_PAIR_NOT_FOUND";
code_str[parseInt("0020", 16)] = "GPIO_GETNUM_WRONG_INDEX";
code_str[parseInt("0021", 16)] = "GPIO_CONFIG_WRONG_INDEX";
code_str[parseInt("0022", 16)] = "GPIO_CONFIG_WRONG_TYPE";
code_str[parseInt("0023", 16)] = "GPIO_UNCONFIG_WRONG_INDEX";
code_str[parseInt("0024", 16)] = "GPIO_GET_CONFIG_WRONG_INDEX";
code_str[parseInt("0025", 16)] = "GPIO_READ_WRONG_INDEX";
code_str[parseInt("0026", 16)] = "GPIO_SET_WRONG_INDEX";
code_str[parseInt("0027", 16)] = "GPIO_SET_WRONG_LEVEL";
code_str[parseInt("0028", 16)] = "GPIO_SET_CANNOT_CHANGE";
code_str[parseInt("0030", 16)] = "LOGGER_RESTORE_CFG_INCOMPLETE";
code_str[parseInt("0031", 16)] = "LOGGER_RESTORE_CFG_FILE_NOT_FOUND";
code_str[parseInt("0032", 16)] = "LOGGER_SAVED_CFG_NOT_UPDATED_INCOMPLETE";
code_str[parseInt("0033", 16)] = "LOGGER_SAVE_CFG_HEAP_EXHAUSTED";
code_str[parseInt("0034", 16)] = "LOGGER_SAVE_CFG_CANNOT_OPEN_FILE";
code_str[parseInt("0035", 16)] = "LOGGER_SAVE_CFG_FS_NOT_AVAILABLE";
code_str[parseInt("0036", 16)] = "LOGGER_INIT_CFG_DEFAULT_CFG";
code_str[parseInt("0037", 16)] = "DIAG_RESTORE_CFG_INCOMPLETE";
code_str[parseInt("0038", 16)] = "DIAG_SAVED_CFG_NOT_UPDATED_INCOMPLETE";
code_str[parseInt("0039", 16)] = "DIAG_SAVE_CFG_FS_NOT_AVAILABLE";
code_str[parseInt("003A", 16)] = "DIAG_SAVE_CFG_CANNOT_OPEN_FILE";
code_str[parseInt("003B", 16)] = "DIAG_INIT_DEFAULT_CFG";
code_str[parseInt("0040", 16)] = "MDNS_START";
code_str[parseInt("0041", 16)] = "MDNS_STOP";
code_str[parseInt("0042", 16)] = "MEM_MON_HEAP_EXHAUSTED";
code_str[parseInt("0043", 16)] = "SNTP_START";
code_str[parseInt("0044", 16)] = "SNTP_STOP";
code_str[parseInt("0045", 16)] = "SNTP_CANNOT_SET_TIMEZONE";
code_str[parseInt("0046", 16)] = "UTILS_CANNOT_PARSE_IP";
code_str[parseInt("0047", 16)] = "ESPBOT_INIT_DEFAULT_CFG";
code_str[parseInt("0048", 16)] = "ESPOT_SET_NAME_TRUNCATED";
code_str[parseInt("0049", 16)] = "ESPBOT_RESTORE_CFG_INCOMPLETE";
code_str[parseInt("004A", 16)] = "ESPBOT_SAVED_CFG_NOT_UPDATED_INCOMPLETE";
code_str[parseInt("004B", 16)] = "ESPBOT_SAVE_CFG_HEAP_EXHAUSTED";
code_str[parseInt("004C", 16)] = "ESPBOT_SAVE_CFG_CANNOT_OPEN_FILE";
code_str[parseInt("004D", 16)] = "ESPBOT_SAVE_CFG_FS_NOT_AVAILABLE";
code_str[parseInt("0050", 16)] = "WIFI_CONNECTED";
code_str[parseInt("0051", 16)] = "WIFI_DISCONNECTED";
code_str[parseInt("0052", 16)] = "WIFI_AUTHMODE_CHANGE";
code_str[parseInt("0053", 16)] = "WIFI_DHCP_TIMEOUT";
code_str[parseInt("0054", 16)] = "WIFI_GOT_IP";
code_str[parseInt("0055", 16)] = "WIFI_OPMODE_CHANGED";
code_str[parseInt("0056", 16)] = "WIFI_STA_CONNECTED";
code_str[parseInt("0057", 16)] = "WIFI_STA_DISCONNECTED";
code_str[parseInt("0058", 16)] = "WIFI_CONNECT_NO_SSID_OR_PASSWORD_AVAILABLE";
code_str[parseInt("0059", 16)] = "WIFI_RESTORE_CFG_INCOMPLETE";
code_str[parseInt("005A", 16)] = "WIFI_RESTORE_CFG_FILE_NOT_FOUND";
code_str[parseInt("005B", 16)] = "WIFI_SAVED_CFG_NOT_UPDATED_INCOMPLETE";
code_str[parseInt("005C", 16)] = "WIFI_SAVE_CFG_HEAP_EXHAUSTED";
code_str[parseInt("005D", 16)] = "WIFI_SAVE_CFG_CANNOT_OPEN_FILE";
code_str[parseInt("005E", 16)] = "WIFI_SAVE_CFG_FS_NOT_AVAILABLE";
code_str[parseInt("005F", 16)] = "WIFI_INIT_CFG_DEFAULT_CFG";
code_str[parseInt("0060", 16)] = "WIFI_TRUNCATING_STRING_TO_31_CHAR";
code_str[parseInt("0061", 16)] = "WIFI_TRUNCATING_STRING_TO_63_CHAR";
code_str[parseInt("0062", 16)] = "WIFI_AP_LIST_HEAP_EXHAUSTED";
code_str[parseInt("0063", 16)] = "WIFI_AP_LIST_CANNOT_COMPLETE_SCAN";
code_str[parseInt("0070", 16)] = "HTTP_CLEAR_BUSY_SENDING_DATA";
code_str[parseInt("0071", 16)] = "HTTP_DELETED_PENDING_RESPONSE";
code_str[parseInt("0072", 16)] = "HTTP_JSON_ERROR_MSG_HEAP_EXHAUSTED";
code_str[parseInt("0073", 16)] = "HTTP_SEND_BUFFER_SEND_QUEUE_FULL";
code_str[parseInt("0074", 16)] = "HTTP_SEND_BUFFER_HEAP_EXHAUSTED";
code_str[parseInt("0075", 16)] = "HTTP_SEND_BUFFER_ERROR";
code_str[parseInt("0076", 16)] = "HTTP_RESPONSE_HEAP_EXHAUSTED";
code_str[parseInt("0077", 16)] = "HTTP_SEND_REMAINING_MSG_RES_QUEUE_FULL";
code_str[parseInt("0078", 16)] = "HTTP_SEND_REMAINING_MSG_HEAP_EXHAUSTED";
code_str[parseInt("0079", 16)] = "HTTP_SEND_RES_QUEUE_FULL";
code_str[parseInt("007A", 16)] = "HTTP_SEND_HEAP_EXHAUSTED";
code_str[parseInt("007B", 16)] = "HTTP_FORMAT_HEADER_HEAP_EXHAUSTED";
code_str[parseInt("007C", 16)] = "HTTP_PARSE_REQUEST_CANNOT_PARSE_EMPTY_MSG";
code_str[parseInt("007D", 16)] = "HTTP_PARSE_REQUEST_HEAP_EXHAUSTED";
code_str[parseInt("007E", 16)] = "HTTP_PARSE_REQUEST_CANNOT_FIND_HTTP_TOKEN";
code_str[parseInt("007F", 16)] = "HTTP_PARSE_REQUEST_CANNOT_FIND_ACC_CTRL_REQ_HEADERS";
code_str[parseInt("0080", 16)] = "HTTP_PARSE_REQUEST_CANNOT_FIND_ORIGIN";
code_str[parseInt("0081", 16)] = "HTTP_PARSE_REQUEST_CANNOT_FIND_CONTENT_START";
code_str[parseInt("0082", 16)] = "HTTP_PARSE_REQUEST_CANNOT_FIND_CONTENT_LEN";
code_str[parseInt("0083", 16)] = "HTTP_SAVE_PENDING_REQUEST_HEAP_EXHAUSTED";
code_str[parseInt("0084", 16)] = "HTTP_SAVE_PENDING_REQUEST_CANNOT_SAVE_REQ";
code_str[parseInt("0085", 16)] = "HTTP_CHECK_PENDING_REQUESTS_CANNOT_FIND_REQ";
code_str[parseInt("0086", 16)] = "HTTP_SAVE_PENDING_RESPONSE_HEAP_EXHAUSTED";
code_str[parseInt("0087", 16)] = "HTTP_SAVE_PENDING_RESPONSE_CANNOT_SAVE_RES";
code_str[parseInt("0088", 16)] = "HTTP_CHECK_PENDING_RESPONSE_CANNOT_FIND_RES";
code_str[parseInt("0089", 16)] = "HTTP_PARSE_RESPONSE_CANNOT_PARSE_EMPTY_MSG";
code_str[parseInt("008A", 16)] = "HTTP_PARSE_RESPONSE_HEAP_EXHAUSTED";
code_str[parseInt("008B", 16)] = "HTTP_PARSE_RESPONSE_CANNOT_FIND_HTTP_TOKEN";
code_str[parseInt("008C", 16)] = "HTTP_PARSE_RESPONSE_CANNOT_FIND_CONTENT_LEN";
code_str[parseInt("008D", 16)] = "HTTP_PARSE_RESPONSE_CANNOT_FIND_CONTENT_RANGE";
code_str[parseInt("008E", 16)] = "HTTP_PARSE_RESPONSE_CANNOT_FIND_RANGE_START";
code_str[parseInt("008F", 16)] = "HTTP_PARSE_RESPONSE_CANNOT_FIND_RANGE_END";
code_str[parseInt("0090", 16)] = "HTTP_PARSE_RESPONSE_CANNOT_FIND_RANGE_SIZE";
code_str[parseInt("0091", 16)] = "HTTP_PARSE_RESPONSE_CANNOT_FIND_CONTENT_START";
code_str[parseInt("0092", 16)] = "WEB_SERVER_START";
code_str[parseInt("0093", 16)] = "WEB_SERVER_STOP";
code_str[parseInt("0094", 16)] = "WEB_SERVER_EMPTY_URL";
code_str[parseInt("00A0", 16)] = "ROUTES_WIFI_SCAN_COMPLETED_FUNCTION_HEAP_EXHAUSTED";
code_str[parseInt("00A1", 16)] = "ROUTES_SEND_REMAINING_MSG_HEAP_EXHAUSTED";
code_str[parseInt("00A2", 16)] = "ROUTES_SEND_REMAINING_MSG_PENDING_RES_QUEUE_FULL";
code_str[parseInt("00A3", 16)] = "ROUTES_RETURN_FILE_HEAP_EXHAUSTED";
code_str[parseInt("00A4", 16)] = "ROUTES_RETURN_FILE_PENDING_RES_QUEUE_FULL";
code_str[parseInt("00A5", 16)] = "ROUTES_PREFLIGHT_RESPONSE_HEAP_EXHAUSTED";
code_str[parseInt("00A6", 16)] = "ROUTES_GET_API_DEBUG_LOG_HEAP_EXHAUSTED";
code_str[parseInt("00A7", 16)] = "ROUTES_GET_API_DEBUG_HEXMEMDUMP_HEAP_EXHAUSTED";
code_str[parseInt("00A8", 16)] = "ROUTES_GET_API_DEBUG_MEMDUMP_HEAP_EXHAUSTED";
code_str[parseInt("00A9", 16)] = "ROUTES_GET_API_DEBUG_MEMINFO_HEAP_EXHAUSTED";
code_str[parseInt("00AA", 16)] = "ROUTES_GET_API_DEBUG_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00AB", 16)] = "ROUTES_POST_API_DEBUG_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00AC", 16)] = "ROUTES_GET_API_ESPBOT_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00AD", 16)] = "ROUTES_POST_API_ESPBOT_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00AE", 16)] = "ROUTES_GET_API_FS_INFO_HEAP_EXHAUSTED";
code_str[parseInt("00AF", 16)] = "ROUTES_GET_API_FILES_LS_HEAP_EXHAUSTED";
code_str[parseInt("00B0", 16)] = "ROUTES_POST_API_GPIO_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00B1", 16)] = "ROUTES_POST_API_GPIO_UNCFG_HEAP_EXHAUSTED";
code_str[parseInt("00B2", 16)] = "ROUTES_GET_API_GPIO_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00B3", 16)] = "ROUTES_GET_API_GPIO_READ_HEAP_EXHAUSTED";
code_str[parseInt("00B4", 16)] = "ROUTES_GET_API_GPIO_SET_HEAP_EXHAUSTED";
code_str[parseInt("00B5", 16)] = "ROUTES_GET_API_OTA_INFO_HEAP_EXHAUSTED";
code_str[parseInt("00B6", 16)] = "ROUTES_GET_API_OTA_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00B7", 16)] = "ROUTES_POST_API_OTA_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00B8", 16)] = "ROUTES_GET_API_WIFI_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00B9", 16)] = "ROUTES_POST_API_WIFI_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00BA", 16)] = "ROUTES_GET_API_WIFI_INFO_HEAP_EXHAUSTED";
code_str[parseInt("00BB", 16)] = "ROUTES_GET_API_DIAG_EVENTS_HEAP_EXHAUSTED";
code_str[parseInt("00BC", 16)] = "ROUTES_GET_API_DIAG_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00BD", 16)] = "ROUTES_POST_API_DIAG_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00BE", 16)] = "ROUTES_GET_SNTP_HEAP_EXHAUSTED";
code_str[parseInt("00BF", 16)] = "ROUTES_POST_API_SNTP_HEAP_EXHAUSTED";
code_str[parseInt("00C0", 16)] = "ROUTES_POST_API_TIMEDATE_HEAP_EXHAUSTED";
code_str[parseInt("00C1", 16)] = "ROUTES_GET_MDNS_HEAP_EXHAUSTED";
code_str[parseInt("00C2", 16)] = "ROUTES_POST_MDNS_HEAP_EXHAUSTED";
code_str[parseInt("00C3", 16)] = "ROUTES_GET_CRON_HEAP_EXHAUSTED";
code_str[parseInt("00C4", 16)] = "ROUTES_POST_CRON_HEAP_EXHAUSTED";
code_str[parseInt("00D0", 16)] = "OTA_INIT_DEFAULT_CFG";
code_str[parseInt("00D1", 16)] = "OTA_SET_PATH_HEAP_EXHAUSTED";
code_str[parseInt("00D2", 16)] = "OTA_SET_CHECK_VERSION_UNKNOWN_VALUE";
code_str[parseInt("00D3", 16)] = "OTA_SET_REBOOT_ON_COMPLETION_UNKNOWN_VALUE";
code_str[parseInt("00D4", 16)] = "OTA_CANNOT_COMPLETE";
code_str[parseInt("00D5", 16)] = "OTA_TIMER_FUNCTION_USERBIN_ID_UNKNOWN";
code_str[parseInt("00D6", 16)] = "OTA_CANNOT_START_UPGRADE";
code_str[parseInt("00D7", 16)] = "OTA_SUCCESSFULLY_COMPLETED";
code_str[parseInt("00D8", 16)] = "OTA_FAILURE";
code_str[parseInt("00D9", 16)] = "OTA_REBOOTING_AFTER_COMPLETION";
code_str[parseInt("00DA", 16)] = "OTA_START_UPGRADE_CALLED_WHILE_OTA_IN_PROGRESS";
code_str[parseInt("00DB", 16)] = "OTA_RESTORE_CFG_FILE_NOT_FOUND";
code_str[parseInt("00DC", 16)] = "OTA_RESTORE_CFG_INCOMPLETE";
code_str[parseInt("00DD", 16)] = "OTA_SAVED_CFG_NOT_UPDATED_INCOMPLETE";
code_str[parseInt("00DE", 16)] = "OTA_SAVE_CFG_FS_NOT_AVAILABLE";
code_str[parseInt("00DF", 16)] = "OTA_SAVE_CFG_CANNOT_OPEN_FILE";
code_str[parseInt("00F0", 16)] = "OTA_SAVE_CFG_HEAP_EXHAUSTED";
code_str[parseInt("00F1", 16)] = "OTA_CHECK_VERSION_UNEXPECTED_WEBCLIENT_STATUS";
code_str[parseInt("00F2", 16)] = "OTA_ASK_VERSION_UNEXPECTED_WEBCLIENT_STATUS";
code_str[parseInt("00F3", 16)] = "OTA_UP_TO_DATE";
code_str[parseInt("00F4", 16)] = "OTA_ENGINE_HEAP_EXHAUSTED";
code_str[parseInt("00F5", 16)] = "OTA_CHECK_VERSION_BAD_FORMAT";
code_str[parseInt("00F6", 16)] = "OTA_CHECK_VERSION_EMPTY_RES";
code_str[parseInt("0100", 16)] = "WEB_CLIENT_SEND_REQ_CANNOT_SEND_REQ";
code_str[parseInt("0101", 16)] = "WEB_CLIENT_ADD_CLIENT_ESPCONN_ASSOCIATION_HEAP_EXHAUSTED";
code_str[parseInt("0102", 16)] = "WEB_CLIENT_ADD_CLIENT_ASSOCIATION_REG_ERROR";
code_str[parseInt("0103", 16)] = "WEB_CLIENT_CONNECT_TIMEOUT";
code_str[parseInt("0104", 16)] = "WEB_CLIENT_SEND_REQ_TIMEOUT";
code_str[parseInt("0105", 16)] = "WEB_CLIENT_RECV_CANNOT_FIND_ESPCONN";
code_str[parseInt("0106", 16)] = "WEB_CLIENT_DISCON_CANNOT_FIND_ESPCONN";
code_str[parseInt("0107", 16)] = "WEB_CLIENT_CONNECTED_CANNOT_FIND_ESPCONN";
code_str[parseInt("0108", 16)] = "WEB_CLIENT_CONNECT_CONN_FAILURE";
code_str[parseInt("0109", 16)] = "WEB_CLIENT_SEND_REQ_HEAP_EXHAUSTED";
code_str[parseInt("0110", 16)] = "SPIFFS_INIT_CANNOT_MOUNT";
code_str[parseInt("0111", 16)] = "SPIFFS_INIT_FS_FORMATTED";
code_str[parseInt("0112", 16)] = "SPIFFS_INIT_CANNOT_FORMAT";
code_str[parseInt("0113", 16)] = "SPIFFS_INIT_FS_MOUNTED";
code_str[parseInt("0114", 16)] = "SPIFFS_INIT_FS_SIZE";
code_str[parseInt("0115", 16)] = "SPIFFS_INIT_FS_USED";
code_str[parseInt("0116", 16)] = "SPIFFS_FORMAT_FS_NOT_INIT";
code_str[parseInt("0117", 16)] = "SPIFFS_FORMAT_FS_FORMATTED";
code_str[parseInt("0118", 16)] = "SPIFFS_FORMAT_CANNOT_FORMAT";
code_str[parseInt("0119", 16)] = "SPIFFS_UNMOUNT_FS_NOT_INIT";
code_str[parseInt("011A", 16)] = "SPIFFS_UNMOUNT_FS_UNMOUNTED";
code_str[parseInt("011B", 16)] = "SPIFFS_LAST_ERROR_FS_NOT_INIT";
code_str[parseInt("011C", 16)] = "SPIFFS_GET_TOTAL_SIZE_FS_NOT_INIT";
code_str[parseInt("011D", 16)] = "SPIFFS_GET_USED_SIZE_FS_NOT_INIT";
code_str[parseInt("011E", 16)] = "SPIFFS_CHECK_FS_NOT_INIT";
code_str[parseInt("011F", 16)] = "SPIFFS_CHECK_SUCCESSFULLY";
code_str[parseInt("0120", 16)] = "SPIFFS_CHECK_ERRORS";
code_str[parseInt("0121", 16)] = "SPIFFS_GET_HANDLER_NOT_INIT";
code_str[parseInt("0122", 16)] = "SPIFFS_FFILE_FS_NOT_AVAILABLE";
code_str[parseInt("0123", 16)] = "SPIFFS_FFILE_FILE_OPEN_ERROR";
code_str[parseInt("0124", 16)] = "SPIFFS_FFILE_NAME_TRUNCATED";
code_str[parseInt("0125", 16)] = "SPIFFS_FFILE_DES_FILE_CLOSE_ERROR";
code_str[parseInt("0126", 16)] = "SPIFFS_GET_NAME_NO_NAME";
code_str[parseInt("0127", 16)] = "SPIFFS_OPEN_FILE_CLOSE_ERROR";
code_str[parseInt("0128", 16)] = "SPIFFS_OPEN_NAME_TRUNCATED";
code_str[parseInt("0129", 16)] = "SPIFFS_OPEN_FILE_OPEN_ERROR";
code_str[parseInt("012A", 16)] = "SPIFFS_OPEN_FS_NOT_AVAILABLE";
code_str[parseInt("012B", 16)] = "SPIFFS_N_READ_FILE_READ_ERROR";
code_str[parseInt("012C", 16)] = "SPIFFS_N_READ_CANNOT_READ";
code_str[parseInt("012D", 16)] = "SPIFFS_N_READ_FS_NOT_AVAILABLE";
code_str[parseInt("012E", 16)] = "SPIFFS_N_READ_FILE_SEEK_ERROR";
code_str[parseInt("012F", 16)] = "SPIFFS_N_APPEND_FILE_WRITE_ERROR";
code_str[parseInt("0130", 16)] = "SPIFFS_N_APPEND_CANNOT_WRITE";
code_str[parseInt("0131", 16)] = "SPIFFS_N_APPEND_FS_NOT_AVAILABLE";
code_str[parseInt("0132", 16)] = "SPIFFS_CLEAR_FILE_CLOSE_ERROR";
code_str[parseInt("0133", 16)] = "SPIFFS_CLEAR_FILE_OPEN_ERROR";
code_str[parseInt("0134", 16)] = "SPIFFS_CLEAR_CANNOT_CLEAR";
code_str[parseInt("0135", 16)] = "SPIFFS_CLEAR_FS_NOT_AVAILABLE";
code_str[parseInt("0136", 16)] = "SPIFFS_REMOVE_FILE_REMOVE_ERROR";
code_str[parseInt("0137", 16)] = "SPIFFS_REMOVE_CANNOT_REMOVE";
code_str[parseInt("0138", 16)] = "SPIFFS_REMOVE_FS_NOT_AVAILABLE";
code_str[parseInt("0139", 16)] = "SPIFFS_FLUSH_FILE_FLUSH_ERROR";
code_str[parseInt("013A", 16)] = "SPIFFS_FLUSH_CANNOT_FLUSH";
code_str[parseInt("013B", 16)] = "SPIFFS_FLUSH_FS_NOT_AVAILABLE";
code_str[parseInt("013C", 16)] = "SPIFFS_EXISTS_FS_NOT_AVAILABLE";
code_str[parseInt("013D", 16)] = "SPIFFS_SIZE_FS_NOT_AVAILABLE";
code_str[parseInt("0140", 16)] = "SPIFFS_FLASH_READ_OUT_OF_BOUNDARY";
code_str[parseInt("0141", 16)] = "SPIFFS_FLASH_READ_ERROR";
code_str[parseInt("0142", 16)] = "SPIFFS_FLASH_READ_TIMEOUT";
code_str[parseInt("0143", 16)] = "SPIFFS_FLASH_WRITE_OUT_OF_BOUNDARY";
code_str[parseInt("0144", 16)] = "SPIFFS_FLASH_WRITE_READ_ERROR";
code_str[parseInt("0145", 16)] = "SPIFFS_FLASH_WRITE_READ_TIMEOUT";
code_str[parseInt("0146", 16)] = "SPIFFS_FLASH_WRITE_WRITE_ERROR";
code_str[parseInt("0147", 16)] = "SPIFFS_FLASH_WRITE_WRITE_TIMEOUT";
code_str[parseInt("0148", 16)] = "SPIFFS_FLASH_ERASE_OUT_OF_BOUNDARY";
code_str[parseInt("0149", 16)] = "SPIFFS_FLASH_ERASE_ERROR";
code_str[parseInt("014A", 16)] = "SPIFFS_FLASH_ERASE_TIMEOUT";
code_str[parseInt("0150", 16)] = "TIMEDATE_RESTORE_CFG_INCOMPLETE";
code_str[parseInt("0151", 16)] = "TIMEDATE_SAVED_CFG_NOT_UPDATED_INCOMPLETE";
code_str[parseInt("0152", 16)] = "TIMEDATE_SAVE_CFG_FS_NOT_AVAILABLE";
code_str[parseInt("0153", 16)] = "TIMEDATE_SAVE_CFG_CANNOT_OPEN_FILE";
code_str[parseInt("0154", 16)] = "TIMEDATE_INIT_DEFAULT_CFG";
code_str[parseInt("0155", 16)] = "TIMEZONE_CHANGED";
code_str[parseInt("0156", 16)] = "TIMEDATE_CHANGED";
code_str[parseInt("0160", 16)] = "MDNS_RESTORE_CFG_INCOMPLETE";
code_str[parseInt("0161", 16)] = "MDNS_SAVED_CFG_NOT_UPDATED_INCOMPLETE";
code_str[parseInt("0162", 16)] = "MDNS_SAVE_CFG_FS_NOT_AVAILABLE";
code_str[parseInt("0163", 16)] = "MDNS_SAVE_CFG_CANNOT_OPEN_FILE";
code_str[parseInt("0164", 16)] = "MDNS_INIT_DEFAULT_CFG";
code_str[parseInt("0170", 16)] = "CRON_RESTORE_CFG_INCOMPLETE";
code_str[parseInt("0171", 16)] = "CRON_SAVED_CFG_NOT_UPDATED_INCOMPLETE";
code_str[parseInt("0172", 16)] = "CRON_SAVE_CFG_FS_NOT_AVAILABLE";
code_str[parseInt("0173", 16)] = "CRON_SAVE_CFG_CANNOT_OPEN_FILE";
code_str[parseInt("0174", 16)] = "CRON_INIT_DEFAULT_CFG";
code_str[parseInt("0175", 16)] = "CRON_START";
code_str[parseInt("0176", 16)] = "CRON_STOP";
code_str[parseInt("0177", 16)] = "CRON_ADD_JOB_CANNOT_COMPLETE";
code_str[parseInt("0178", 16)] = "CRON_ADD_JOB_HEAP_EXHAUSTED";
code_str[parseInt("0179", 16)] = "CRON_ENABLED";
code_str[parseInt("017A", 16)] = "CRON_DISABLED";
code_str[parseInt("0100", 16)] = "DHT_HEAP_EXHAUSTED";
code_str[parseInt("0101", 16)] = "DHT_READING_TIMEOUT";
code_str[parseInt("0102", 16)] = "DHT_READING_CHECKSUM_ERR";
code_str[parseInt("0103", 16)] = "DHT_READ_HEAP_EXHAUSTED";
code_str[parseInt("0104", 16)] = "MAX6675_THERMOCOUPLE_DISCONNECTED";
code_str[parseInt("0105", 16)] = "MAX6675_HEAP_EXHAUSTED";
return code_str[parseInt(code, 16)]; }
