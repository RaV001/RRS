#include    <switch-label.h>
#include    <QMouseEvent>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
SwitchLabel::SwitchLabel(QWidget *parent) : QLabel(parent)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
SwitchLabel::~SwitchLabel()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void SwitchLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        emit popUpMenu();
    }
}