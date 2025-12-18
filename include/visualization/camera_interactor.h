#pragma once

#include <QObject>

class QVTKOpenGLNativeWidget;
class QKeyEvent;

/**
 * @brief 相机交互控制器
 * 处理键盘事件以控制 VTK 相机移动
 */
class CameraInteractor : public QObject
{
    Q_OBJECT
public:
    explicit CameraInteractor(QVTKOpenGLNativeWidget *widget, QObject *parent = nullptr);
    ~CameraInteractor();

protected:
    /** @brief 事件过滤器，用于拦截键盘事件 */
    bool eventFilter(QObject *obj, QEvent *ev) override;

private:
    /** @brief 处理键盘按键事件 */
    void handleKey(QKeyEvent *ke);

    QVTKOpenGLNativeWidget *widget_ = nullptr;
    double baseStep_ = 0.5;
};
