/****************************************************************************
** Meta object code from reading C++ file 'camera_publisher_tool.h'
**
** Created: Thu Apr 10 15:24:31 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "camera_publisher_tool.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'camera_publisher_tool.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_rviz_camera_publisher__CameraPublisherTool[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      44,   43,   43,   43, 0x08,
      64,   43,   43,   43, 0x08,
      86,   43,   43,   43, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_rviz_camera_publisher__CameraPublisherTool[] = {
    "rviz_camera_publisher::CameraPublisherTool\0"
    "\0UpdateOutputTopic()\0UpdatePublishButton()\0"
    "Update()\0"
};

void rviz_camera_publisher::CameraPublisherTool::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CameraPublisherTool *_t = static_cast<CameraPublisherTool *>(_o);
        switch (_id) {
        case 0: _t->UpdateOutputTopic(); break;
        case 1: _t->UpdatePublishButton(); break;
        case 2: _t->Update(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData rviz_camera_publisher::CameraPublisherTool::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject rviz_camera_publisher::CameraPublisherTool::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_rviz_camera_publisher__CameraPublisherTool,
      qt_meta_data_rviz_camera_publisher__CameraPublisherTool, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &rviz_camera_publisher::CameraPublisherTool::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *rviz_camera_publisher::CameraPublisherTool::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *rviz_camera_publisher::CameraPublisherTool::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_camera_publisher__CameraPublisherTool))
        return static_cast<void*>(const_cast< CameraPublisherTool*>(this));
    typedef rviz::Panel QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int rviz_camera_publisher::CameraPublisherTool::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rviz::Panel QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
