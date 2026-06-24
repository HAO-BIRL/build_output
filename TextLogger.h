/**
 * @file   TextLogger.h
 * @brief  分级文本日志记录器（事件 / 警告 / 错误调试用）
 *
 * 功能
 * ─────────────────────────────────────────────────────────────
 * - 四个日志级别：DEBUG / INFO / WARN / ERROR
 * - 每条日志自动加时间戳和级别标签
 * - 日志先缓冲在内存，save() 时写入文件；
 *   echo_console = true 时同步打印到终端（便于实时观察）
 * - 线程安全（互斥锁保护缓冲区）
 *
 * 适用场景
 * ─────────────────────────────────────────────────────────────
 * 记录控制模式切换、通信异常、标定结果、关键参数等文字信息；
 * 高频数值数据请使用 DataLogger。
 *
 * 典型用法
 * ─────────────────────────────────────────────────────────────
 *   TextLogger tlog;
 *   tlog.info("控制器已初始化，dt = 5 ms");
 *   tlog.warn("速度超限：dq = 12.3 rad/s");
 *   tlog.error("readFeedback 超时，第 42 周期");
 *
 *   tlog.save();   // → logs/2025-01-01_12-00-00.log
 */
#pragma once

#include <string>
#include <vector>
#include <mutex>

enum class LogLevel
{
    DEBUG = 0,
    INFO,
    WARN,
    ERROR
};

class TextLogger
{
public:
    struct Config
    {
        std::string dir          = "logs";       ///< 日志输出目录
        LogLevel    min_level    = LogLevel::DEBUG; ///< 低于此级别的日志不记录
        bool        echo_console = true;         ///< 是否同步打印到终端
    };

    /** @brief 使用默认 Config 构造 */
    TextLogger();

    /** @brief 使用自定义 Config 构造 */
    explicit TextLogger(const Config& cfg);

    /**
     * @brief 记录一条日志（线程安全）
     * @param level 日志级别
     * @param msg   日志内容
     */
    void log(LogLevel level, const std::string& msg);

    /** @brief 调试信息（详细执行过程，正式运行可关闭）*/
    void debug(const std::string& msg);

    /** @brief 普通信息（关键事件、状态切换）*/
    void info (const std::string& msg);

    /** @brief 警告（非致命异常，程序可继续运行）*/
    void warn (const std::string& msg);

    /** @brief 错误（需要关注的严重问题）*/
    void error(const std::string& msg);

    /**
     * @brief 将缓冲区写入日志文件
     * @param filename 文件名（含扩展名）；为空则按时间自动命名
     * @return true = 写入成功
     */
    bool save(const std::string& filename = "") const;

    /** @brief 清空缓冲区 */
    void reset();

    int count() const;   ///< 已缓冲的日志条数

private:
    Config                   cfg_;
    std::vector<std::string> lines_;   ///< 已格式化的日志行缓冲
    mutable std::mutex       mtx_;

    static const char* levelTag(LogLevel level);
};
