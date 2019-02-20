#include    "hud.h"
#include    "filesystem.h"

#include    <osg/Geode>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
HUD::HUD(int width, int height, QObject *parent) : QObject (parent)
  , camera(nullptr)
  , scene(new osg::Switch)
  , view(new osgViewer::View)
{
    FileSystem &fs = FileSystem::getInstance();
    fontPath = fs.getFontsDir() + fs.separator() + "arial.ttf";

    camera = createCamera(width, height);    
    camera->setViewport(0, 0, width, height);
    view->setCamera(camera.get());
}

HUD::~HUD()
{

}

osg::Camera *HUD::getCamera()
{
    return camera.get();
}

osg::Switch *HUD::getScene()
{
    return scene.get();
}

osgViewer::View *HUD::getView()
{
    return view.get();
}

osg::Camera *HUD::createCamera(int width, int height)
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0, width, 0, height));
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setRenderOrder(osg::Camera::POST_RENDER);
    camera->setAllowEventFocus(false);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    geode->addDrawable(createText(osg::Vec3(width / 2, height / 2, 0), L"Всё заебись!!!", 60.0f));

    scene->addChild(geode.get(), true);
    camera->addChild(geode.get());

    return camera.release();
}

osgText::Text *HUD::createText(const osg::Vec3 &position, std::wstring content, float size)
{
    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setFont(fontPath);
    text->setCharacterSize(size);
    text->setPosition(position);
    text->setText(content.c_str());
    text->setAlignment(osgText::Text::CENTER_CENTER);
    text->setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));

    return text.release();
}

KeyboardHUDHandler::KeyboardHUDHandler(osg::Switch *switchNode)
    : switchNode(switchNode)
{

}

bool KeyboardHUDHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    Q_UNUSED(aa)

    switch (ea.getEventType())
    {
    case osgGA::GUIEventAdapter::KEYDOWN:

        switch (ea.getKey())
        {
        case osgGA::GUIEventAdapter::KEY_F1:
            switchNode->setValue(0, !switchNode->getValue(0));
            break;
        }

        break;

    default:

        break;
    }

    return false;
}
