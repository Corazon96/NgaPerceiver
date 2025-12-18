#include "visualization/camera_interactor.h"

#include <QKeyEvent>
#include <QDebug>

#include <QVTKOpenGLNativeWidget.h>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkRendererCollection.h>
#include <vtkMath.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPointPicker.h>

/**
 * @brief 自定义交互样式：增强鼠标交互体验
 * 滚轮缩放更平滑
 */
class LingerInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static LingerInteractorStyle* New();
    vtkTypeMacro(LingerInteractorStyle, vtkInteractorStyleTrackballCamera);

    void OnMouseMove() override
    {
        // 1. 计算速度倍率
        double speed = 1.0;
        if (this->Interactor->GetShiftKey()) speed = 5.0;
        if (this->Interactor->GetControlKey()) speed = 0.1;

        // 2. 应用到 MotionFactor (影响旋转、缩放)
        double originalFactor = this->MotionFactor;
        this->MotionFactor *= speed;

        // 3. 应用到 Pan (平移)
        // Pan 的速度通常与鼠标移动距离 1:1 对应。
        // 为了改变速度，我们欺骗父类，让它以为鼠标移动了更多/更少的距离。
        int* pos = this->Interactor->GetEventPosition();
        int* lastPos = this->Interactor->GetLastEventPosition();
        int savedLastPos[2] = {lastPos[0], lastPos[1]}; // 备份

        if (this->State == VTKIS_PAN && speed != 1.0)
        {
            int dx = pos[0] - lastPos[0];
            int dy = pos[1] - lastPos[1];
            
            // 构造一个新的 LastPosition，使得 (Pos - NewLast) = (Pos - Last) * speed
            int newLastX = pos[0] - static_cast<int>(dx * speed);
            int newLastY = pos[1] - static_cast<int>(dy * speed);
            
            this->Interactor->SetLastEventPosition(newLastX, newLastY);
        }

        // 4. 调用父类处理
        vtkInteractorStyleTrackballCamera::OnMouseMove();

        // 5. 恢复状态
        this->MotionFactor = originalFactor;
        if (this->State == VTKIS_PAN && speed != 1.0)
        {
            this->Interactor->SetLastEventPosition(savedLastPos[0], savedLastPos[1]);
        }
    }

    void OnMouseWheelForward() override
    {
        double originalFactor = this->MotionFactor;
        if (this->Interactor->GetShiftKey()) this->MotionFactor *= 5.0;
        if (this->Interactor->GetControlKey()) this->MotionFactor *= 0.1;
        
        vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
        
        this->MotionFactor = originalFactor;
    }

    void OnMouseWheelBackward() override
    {
        double originalFactor = this->MotionFactor;
        if (this->Interactor->GetShiftKey()) this->MotionFactor *= 5.0;
        if (this->Interactor->GetControlKey()) this->MotionFactor *= 0.1;
        
        vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
        
        this->MotionFactor = originalFactor;
    }
};

vtkStandardNewMacro(LingerInteractorStyle);

#include <cmath>

CameraInteractor::CameraInteractor(QVTKOpenGLNativeWidget *widget, QObject *parent)
    : QObject(parent), widget_(widget), baseStep_(2.0)
{
    if (widget_)
    {
        // 仅安装事件过滤器；焦点由控件所有者处理
        widget_->installEventFilter(this);
        
        // 注意：不要在构造函数中立即设置 InteractorStyle。
        // 因为此时 Renderer 可能还没有调用 setRenderWindow。
        // 这里的 renderWindow 可能是临时的，或者稍后会被 Renderer 替换掉。
        // 我们将在 eventFilter 中进行惰性初始化 (Lazy Initialization)。
    }
}

CameraInteractor::~CameraInteractor() {}

bool CameraInteractor::eventFilter(QObject *obj, QEvent *ev)
{
    if (!widget_ || obj != widget_)
        return QObject::eventFilter(obj, ev);

    // 惰性初始化
    // 每次有相关事件时，检查当前的 InteractorStyle 是否是我们自定义的。
    // 如果 RenderWindow 被替换了（例如在 Renderer::init 中），这里可以重新应用 Style。
    if (ev->type() == QEvent::MouseButtonPress || ev->type() == QEvent::KeyPress || ev->type() == QEvent::Enter)
    {
        if (widget_->renderWindow() && widget_->renderWindow()->GetInteractor())
        {
            vtkRenderWindowInteractor* iren = widget_->renderWindow()->GetInteractor();
            // 检查当前样式是否是 LingerInteractorStyle
            if (!LingerInteractorStyle::SafeDownCast(iren->GetInteractorStyle()))
            {
                vtkSmartPointer<LingerInteractorStyle> style = vtkSmartPointer<LingerInteractorStyle>::New();
                // 尝试获取第一个 Renderer
                vtkRenderer* ren = widget_->renderWindow()->GetRenderers()->GetFirstRenderer();
                if (ren) {
                    style->SetDefaultRenderer(ren);
                }
                iren->SetInteractorStyle(style);
            }
        }
        
        // 确保点击时获取焦点，以便接收键盘事件
        if (ev->type() == QEvent::MouseButtonPress) {
            widget_->setFocus();
        }
    }

    if (ev->type() == QEvent::KeyPress)
    {
        QKeyEvent *ke = static_cast<QKeyEvent *>(ev);
        int k = ke->key();

        switch (k)
        {
        case Qt::Key_W:
        case Qt::Key_A:
        case Qt::Key_S:
        case Qt::Key_D:
        case Qt::Key_Q:
        case Qt::Key_E:
        case Qt::Key_R:
            handleKey(ke);
            return true;
        default:
            return QObject::eventFilter(obj, ev);
        }
    }
    return QObject::eventFilter(obj, ev);
}

void CameraInteractor::handleKey(QKeyEvent *ke)
{
    if (!widget_)
        return;
    
    // 再次检查 RenderWindow，防止空指针
    vtkRenderWindow *rw = widget_->renderWindow();
    if (!rw) return;
    
    vtkRenderer *ren = rw->GetRenderers()->GetFirstRenderer();
    if (!ren) return;
    
    vtkCamera *cam = ren->GetActiveCamera();
    if (!cam) return;

    double pos[3], fp[3], up[3];
    cam->GetPosition(pos);
    cam->GetFocalPoint(fp);
    cam->GetViewUp(up);

    double forward[3] = {fp[0] - pos[0], fp[1] - pos[1], fp[2] - pos[2]};
    vtkMath::Normalize(forward);
    double right[3];
    vtkMath::Cross(forward, up, right);
    vtkMath::Normalize(right);

    double step = baseStep_;
    if (ke->modifiers() & Qt::ShiftModifier)
    {
        step *= 5.0; 
    }
        
    if (ke->modifiers() & Qt::ControlModifier)
    {
        step *= 0.1; 
    }

    int k = ke->key();
    bool moved = false;
    if (k == Qt::Key_W)
    {
        for (int i = 0; i < 3; ++i)
        {
            pos[i] += forward[i] * step;
            fp[i] += forward[i] * step;
        }
        moved = true;
    }
    else if (k == Qt::Key_S)
    {
        for (int i = 0; i < 3; ++i)
        {
            pos[i] -= forward[i] * step;
            fp[i] -= forward[i] * step;
        }
        moved = true;
    }
    else if (k == Qt::Key_A)
    {
        for (int i = 0; i < 3; ++i)
        {
            pos[i] -= right[i] * step;
            fp[i] -= right[i] * step;
        }
        moved = true;
    }
    else if (k == Qt::Key_D)
    {
        for (int i = 0; i < 3; ++i)
        {
            pos[i] += right[i] * step;
            fp[i] += right[i] * step;
        }
        moved = true;
    }
    else if (k == Qt::Key_Q)
    {
        for (int i = 0; i < 3; ++i)
        {
            pos[i] += up[i] * step;
            fp[i] += up[i] * step;
        }
        moved = true;
    }
    else if (k == Qt::Key_E)
    {
        for (int i = 0; i < 3; ++i)
        {
            pos[i] -= up[i] * step;
            fp[i] -= up[i] * step;
        }
        moved = true;
    }
    else if (k == Qt::Key_R)
    {
        ren->ResetCamera();
        ren->ResetCameraClippingRange();
        rw->Render();
        return;
    }

    if (moved)
    {
        cam->SetPosition(pos);
        cam->SetFocalPoint(fp);
        ren->ResetCameraClippingRange();
        rw->Render();
    }
}
