/****************************************************************************
** Meta object code from reading C++ file 'panelchild.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../panelchild.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'panelchild.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Panelchild_t {
    QByteArrayData data[9];
    char stringdata0[103];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Panelchild_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Panelchild_t qt_meta_stringdata_Panelchild = {
    {
QT_MOC_LITERAL(0, 0, 10), // "Panelchild"
QT_MOC_LITERAL(1, 11, 11), // "slotAddItem"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 16), // "const QMimeData*"
QT_MOC_LITERAL(4, 41, 8), // "mimeData"
QT_MOC_LITERAL(5, 50, 12), // "QPushButton*"
QT_MOC_LITERAL(6, 63, 14), // "slotRemoveItem"
QT_MOC_LITERAL(7, 78, 16), // "slotExchangeItem"
QT_MOC_LITERAL(8, 95, 7) // "objName"

    },
    "Panelchild\0slotAddItem\0\0const QMimeData*\0"
    "mimeData\0QPushButton*\0slotRemoveItem\0"
    "slotExchangeItem\0objName"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Panelchild[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   29,    2, 0x0a /* Public */,
       6,    1,   34,    2, 0x0a /* Public */,
       7,    2,   37,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    2,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 3,    8,    4,

       0        // eod
};

void Panelchild::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Panelchild *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->slotAddItem((*reinterpret_cast< const QMimeData*(*)>(_a[1])),(*reinterpret_cast< QPushButton*(*)>(_a[2]))); break;
        case 1: _t->slotRemoveItem((*reinterpret_cast< const QMimeData*(*)>(_a[1]))); break;
        case 2: _t->slotExchangeItem((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QMimeData*(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QPushButton* >(); break;
            }
            break;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Panelchild::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_Panelchild.data,
    qt_meta_data_Panelchild,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Panelchild::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Panelchild::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Panelchild.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int Panelchild::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
