/****************************************************************************
** Meta object code from reading C++ file 'subgphitem.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../subgphitem.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'subgphitem.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SubGphItem_t {
    QByteArrayData data[12];
    char stringdata0[132];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SubGphItem_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SubGphItem_t qt_meta_stringdata_SubGphItem = {
    {
QT_MOC_LITERAL(0, 0, 10), // "SubGphItem"
QT_MOC_LITERAL(1, 11, 10), // "addGphItem"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 16), // "const QMimeData*"
QT_MOC_LITERAL(4, 40, 8), // "mimeData"
QT_MOC_LITERAL(5, 49, 12), // "QPushButton*"
QT_MOC_LITERAL(6, 62, 13), // "removeGphItem"
QT_MOC_LITERAL(7, 76, 15), // "exchangeGphItem"
QT_MOC_LITERAL(8, 92, 7), // "objName"
QT_MOC_LITERAL(9, 100, 14), // "signIsSelected"
QT_MOC_LITERAL(10, 115, 11), // "SubGphItem*"
QT_MOC_LITERAL(11, 127, 4) // "item"

    },
    "SubGphItem\0addGphItem\0\0const QMimeData*\0"
    "mimeData\0QPushButton*\0removeGphItem\0"
    "exchangeGphItem\0objName\0signIsSelected\0"
    "SubGphItem*\0item"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SubGphItem[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   34,    2, 0x06 /* Public */,
       6,    1,   39,    2, 0x06 /* Public */,
       7,    2,   42,    2, 0x06 /* Public */,
       9,    1,   47,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    2,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 3,    8,    4,
    QMetaType::Void, 0x80000000 | 10,   11,

       0        // eod
};

void SubGphItem::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SubGphItem *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->addGphItem((*reinterpret_cast< const QMimeData*(*)>(_a[1])),(*reinterpret_cast< QPushButton*(*)>(_a[2]))); break;
        case 1: _t->removeGphItem((*reinterpret_cast< const QMimeData*(*)>(_a[1]))); break;
        case 2: _t->exchangeGphItem((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QMimeData*(*)>(_a[2]))); break;
        case 3: _t->signIsSelected((*reinterpret_cast< SubGphItem*(*)>(_a[1]))); break;
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
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< SubGphItem* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (SubGphItem::*)(const QMimeData * , QPushButton * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SubGphItem::addGphItem)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (SubGphItem::*)(const QMimeData * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SubGphItem::removeGphItem)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (SubGphItem::*)(const QString , const QMimeData * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SubGphItem::exchangeGphItem)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (SubGphItem::*)(SubGphItem * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SubGphItem::signIsSelected)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SubGphItem::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_SubGphItem.data,
    qt_meta_data_SubGphItem,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SubGphItem::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SubGphItem::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SubGphItem.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "QGraphicsPathItem"))
        return static_cast< QGraphicsPathItem*>(this);
    return QObject::qt_metacast(_clname);
}

int SubGphItem::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void SubGphItem::addGphItem(const QMimeData * _t1, QPushButton * _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SubGphItem::removeGphItem(const QMimeData * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SubGphItem::exchangeGphItem(const QString _t1, const QMimeData * _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void SubGphItem::signIsSelected(SubGphItem * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
