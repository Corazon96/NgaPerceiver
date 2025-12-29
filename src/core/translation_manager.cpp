#include "core/translation_manager.h"
#include <QDir>
#include <QCoreApplication>
#include <QLibraryInfo>
#include <QMessageBox>
#include <QSettings>
#include <spdlog/spdlog.h>

TranslationManager& TranslationManager::instance()
{
    static TranslationManager instance;
    return instance;
}

TranslationManager::TranslationManager()
    : appTranslator_(std::make_unique<QTranslator>())
    , qtTranslator_(std::make_unique<QTranslator>())
{
}

TranslationManager::~TranslationManager() = default;

void TranslationManager::initialize(QApplication* app)
{
    app_ = app;
    
    // 从配置加载语言设置
    loadFromConfig();
    
    // 应用当前语言
    setLanguage(currentLang_);
}

bool TranslationManager::setLanguage(const QString& langCode)
{
    if (!app_) {
        spdlog::error("TranslationManager not initialized");
        return false;
    }

    QString normalizedCode = langCode.toLower();
    if (normalizedCode == "zh" || normalizedCode == "zh_cn" || normalizedCode == "chinese") {
        normalizedCode = "zh_CN";
    } else if (normalizedCode == "en" || normalizedCode == "english") {
        normalizedCode = "en";
    }

    // 移除之前的翻译器
    if (appTranslator_->isEmpty() == false) {
        app_->removeTranslator(appTranslator_.get());
    }
    if (qtTranslator_->isEmpty() == false) {
        app_->removeTranslator(qtTranslator_.get());
    }

    // 查找并加载应用翻译文件
    QString qmFile = findTranslationFile(normalizedCode);
    if (!qmFile.isEmpty() && appTranslator_->load(qmFile)) {
        app_->installTranslator(appTranslator_.get());
        spdlog::info("Loaded translation file: {}", qmFile.toStdString());
    } else if (normalizedCode != "en") {
        spdlog::warn("Failed to load translation file for: {}", normalizedCode.toStdString());
    }

    // 加载 Qt 标准翻译（对话框按钮等）
    QString qtLangCode = (normalizedCode == "zh_CN") ? "zh_CN" : "en";
    QString qtQmPath = QLibraryInfo::location(QLibraryInfo::TranslationsPath);
    if (qtTranslator_->load("qt_" + qtLangCode, qtQmPath)) {
        app_->installTranslator(qtTranslator_.get());
        spdlog::info("Loaded Qt translation: qt_{}", qtLangCode.toStdString());
    }

    currentLang_ = normalizedCode;
    
    // 发射语言切换信号
    emit languageChanged(currentLang_);
    
    spdlog::info("Language changed to: {}", currentLang_.toStdString());
    return true;
}

bool TranslationManager::setLanguage(Language lang)
{
    switch (lang) {
        case Language::Chinese:
            return setLanguage("zh_CN");
        case Language::English:
        default:
            return setLanguage("en");
    }
}

TranslationManager::Language TranslationManager::currentLanguageEnum() const
{
    if (currentLang_ == "zh_CN") {
        return Language::Chinese;
    }
    return Language::English;
}

QList<QPair<QString, QString>> TranslationManager::supportedLanguages()
{
    return {
        {"en", "English"},
        {"zh_CN", QStringLiteral("简体中文")}
    };
}

QString TranslationManager::findTranslationFile(const QString& langCode) const
{
    // 搜索路径优先级
    QStringList searchPaths = {
        QCoreApplication::applicationDirPath() + "/translations",
        QCoreApplication::applicationDirPath() + "/../translations",
        QDir::currentPath() + "/translations",
        ":/translations"  // Qt 资源文件
    };

    QString fileName = QString("linger_%1.qm").arg(langCode);

    for (const QString& path : searchPaths) {
        QString fullPath = QDir(path).filePath(fileName);
        if (QFile::exists(fullPath)) {
            spdlog::debug("Found translation file: {}", fullPath.toStdString());
            return fullPath;
        }
    }

    spdlog::warn("Translation file not found: {}", fileName.toStdString());
    return QString();
}

void TranslationManager::loadFromConfig()
{
    // 使用 QSettings 存储语言偏好（独立于 app_config.json）
    QSettings settings(
        QCoreApplication::applicationDirPath() + "/config/ui_settings.ini",
        QSettings::IniFormat
    );
    currentLang_ = settings.value("language", "en").toString();
    spdlog::info("Loaded language from config: {}", currentLang_.toStdString());
}

void TranslationManager::saveToConfig()
{
    // 使用 QSettings 存储语言偏好
    QSettings settings(
        QCoreApplication::applicationDirPath() + "/config/ui_settings.ini",
        QSettings::IniFormat
    );
    settings.setValue("language", currentLang_);
    settings.sync();
    spdlog::info("Saved language to config: {}", currentLang_.toStdString());
}
