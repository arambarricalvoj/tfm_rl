/****************************************************************************
** Meta object code from reading C++ file 'Create3Hmi.hh'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/irobot_create_gz/irobot_create_gz_plugins/Create3Hmi/Create3Hmi.hh"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Create3Hmi.hh' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_gz__gui__Create3Hmi_t {
    QByteArrayData data[9];
    char stringdata0[95];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gz__gui__Create3Hmi_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gz__gui__Create3Hmi_t qt_meta_stringdata_gz__gui__Create3Hmi = {
    {
QT_MOC_LITERAL(0, 0, 19), // "gz::gui::Create3Hmi"
QT_MOC_LITERAL(1, 20, 16), // "NamespaceChanged"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 12), // "SetNamespace"
QT_MOC_LITERAL(4, 51, 5), // "_name"
QT_MOC_LITERAL(5, 57, 15), // "OnCreate3Button"
QT_MOC_LITERAL(6, 73, 6), // "button"
QT_MOC_LITERAL(7, 80, 9), // "Namespace"
QT_MOC_LITERAL(8, 90, 4) // "name"

    },
    "gz::gui::Create3Hmi\0NamespaceChanged\0"
    "\0SetNamespace\0_name\0OnCreate3Button\0"
    "button\0Namespace\0name"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gz__gui__Create3Hmi[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       1,   42, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    1,   35,    2, 0x0a /* Public */,
       5,    1,   38,    2, 0x09 /* Protected */,

 // methods: name, argc, parameters, tag, flags
       7,    0,   41,    2, 0x02 /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void, QMetaType::Int,    6,

 // methods: parameters
    QMetaType::QString,

 // properties: name, type, flags
       8, QMetaType::QString, 0x00495003,

 // properties: notify_signal_id
       0,

       0        // eod
};

void gz::gui::Create3Hmi::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Create3Hmi *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->NamespaceChanged(); break;
        case 1: _t->SetNamespace((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->OnCreate3Button((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 3: { QString _r = _t->Namespace();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Create3Hmi::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Create3Hmi::NamespaceChanged)) {
                *result = 0;
                return;
            }
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<Create3Hmi *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QString*>(_v) = _t->Namespace(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<Create3Hmi *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: _t->SetNamespace(*reinterpret_cast< QString*>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

QT_INIT_METAOBJECT const QMetaObject gz::gui::Create3Hmi::staticMetaObject = { {
    QMetaObject::SuperData::link<Plugin::staticMetaObject>(),
    qt_meta_stringdata_gz__gui__Create3Hmi.data,
    qt_meta_data_gz__gui__Create3Hmi,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *gz::gui::Create3Hmi::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gz::gui::Create3Hmi::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gz__gui__Create3Hmi.stringdata0))
        return static_cast<void*>(this);
    return Plugin::qt_metacast(_clname);
}

int gz::gui::Create3Hmi::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Plugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 1;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void gz::gui::Create3Hmi::NamespaceChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
