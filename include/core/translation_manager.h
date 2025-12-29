#pragma once

#include <QObject>
#include <QString>
#include <QTranslator>
#include <QApplication>
#include <memory>

/**
 * @brief 翻译管理器 - 负责应用程序的国际化支持
 * 
 * 支持的语言:
 * - zh_CN: 简体中文
 * - en: English
 * 
 * 使用方法:
 * 1. 在 main() 中初始化: TranslationManager::instance().initialize(app);
 * 2. 切换语言: TranslationManager::instance().setLanguage("zh_CN");
 * 3. 在 UI 代码中使用 tr() 宏标记需要翻译的字符串
 */
class TranslationManager : public QObject
{
    Q_OBJECT

public:
    /// 支持的语言枚举
    enum class Language {
        English,    ///< 英文 (默认)
        Chinese     ///< 简体中文
    };

    /// 获取单例实例
    static TranslationManager& instance();

    /// 初始化翻译管理器（在 QApplication 创建后调用）
    void initialize(QApplication* app);

    /// 设置当前语言
    /// @param langCode 语言代码: "en", "zh_CN"
    /// @return 是否成功切换
    bool setLanguage(const QString& langCode);

    /// 设置当前语言（枚举版本）
    bool setLanguage(Language lang);

    /// 获取当前语言代码
    QString currentLanguage() const { return currentLang_; }

    /// 获取当前语言枚举
    Language currentLanguageEnum() const;

    /// 获取支持的语言列表 (代码, 显示名称)
    static QList<QPair<QString, QString>> supportedLanguages();

    /// 从配置加载语言设置
    void loadFromConfig();

    /// 保存语言设置到配置
    void saveToConfig();

signals:
    /// 语言切换完成信号（UI 可连接此信号来刷新界面）
    void languageChanged(const QString& langCode);

private:
    TranslationManager();
    ~TranslationManager();
    TranslationManager(const TranslationManager&) = delete;
    TranslationManager& operator=(const TranslationManager&) = delete;

    /// 查找翻译文件路径
    QString findTranslationFile(const QString& langCode) const;

    QApplication* app_ = nullptr;
    std::unique_ptr<QTranslator> appTranslator_;
    std::unique_ptr<QTranslator> qtTranslator_;
    QString currentLang_ = "en";
};
