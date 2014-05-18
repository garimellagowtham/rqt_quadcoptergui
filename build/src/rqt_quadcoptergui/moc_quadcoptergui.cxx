/****************************************************************************
** Meta object code from reading C++ file 'quadcoptergui.h'
**
** Created: Mon May 12 08:05:36 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/rqt_quadcoptergui/quadcoptergui.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'quadcoptergui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_rqt_quadcoptergui__QuadcopterGui[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      34,   33,   33,   33, 0x09,
      51,   33,   33,   33, 0x09,
      65,   33,   33,   33, 0x09,
      81,   33,   33,   33, 0x09,
     105,   33,   33,   33, 0x09,
     126,   33,   33,   33, 0x09,
     150,   33,   33,   33, 0x09,
     180,   33,   33,   33, 0x09,
     203,   33,   33,   33, 0x09,
     230,   33,   33,   33, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_rqt_quadcoptergui__QuadcopterGui[] = {
    "rqt_quadcoptergui::QuadcopterGui\0\0"
    "wrappertakeoff()\0wrapperLand()\0"
    "wrapperDisarm()\0wrapperimu_recalib(int)\0"
    "perturb_control(int)\0integrator_control(int)\0"
    "enable_disablecontroller(int)\0"
    "enable_disablelog(int)\0"
    "wrapper_estthrustbias(int)\0RefreshGui()\0"
};

void rqt_quadcoptergui::QuadcopterGui::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QuadcopterGui *_t = static_cast<QuadcopterGui *>(_o);
        switch (_id) {
        case 0: _t->wrappertakeoff(); break;
        case 1: _t->wrapperLand(); break;
        case 2: _t->wrapperDisarm(); break;
        case 3: _t->wrapperimu_recalib((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->perturb_control((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->integrator_control((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->enable_disablecontroller((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->enable_disablelog((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->wrapper_estthrustbias((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->RefreshGui(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData rqt_quadcoptergui::QuadcopterGui::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject rqt_quadcoptergui::QuadcopterGui::staticMetaObject = {
    { &rqt_gui_cpp::Plugin::staticMetaObject, qt_meta_stringdata_rqt_quadcoptergui__QuadcopterGui,
      qt_meta_data_rqt_quadcoptergui__QuadcopterGui, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &rqt_quadcoptergui::QuadcopterGui::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *rqt_quadcoptergui::QuadcopterGui::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *rqt_quadcoptergui::QuadcopterGui::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_quadcoptergui__QuadcopterGui))
        return static_cast<void*>(const_cast< QuadcopterGui*>(this));
    typedef rqt_gui_cpp::Plugin QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int rqt_quadcoptergui::QuadcopterGui::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rqt_gui_cpp::Plugin QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
