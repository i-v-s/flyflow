#ifndef VECTPL_H
#define VECTPL_H
#include <assert.h>

namespace vtp {

/// Шаблон класса с меткой
template<Enum id, typename ItemT>
class Tag
{
public:
    typedef ItemT Item;
    Item item;
    enum
    {
        tag = id
    };
};

/// Шаблон вектора
template<typename... Types>
class Vector;

/// Специализация пустого вектора
template<> class Vector<>
{
public:
    template<class O>
    constexpr inline Vector<> & operator += (const O &) { return *this; }
    //constexpr bool has(Enum) const { return false; }
    template<Enum tag> static constexpr bool has() { return false; }
    template<class Other> static constexpr bool subsetOf() { return true; }
    constexpr int &getItem() const { return *(int *) nullptr; }
    //template<Enum tag> constexpr get
};

//-------------------------- FindItem ------------------------------------------------------------------//
/// Шаблон для поиска элемента вектора по метке
template<Enum tag, class Item> struct FindItem;

/// Шаблон FindItem с указанием найдена метка или нет
template<bool found, Enum tag, class Item> struct FindItemBool;

/// Специализация FindItem: ничего не найдено
template<Enum tag>
struct FindItem<tag, Vector<>>
{
    typedef Vector<> Vector;
    typedef int Item;
};

/// Специализация FindItemBool: результат не найден, продолжаем поиск в остатке вектора
template<Enum tag, class ItemT, class... Others>
struct FindItemBool<false, tag, Vector<ItemT, Others...>> : public FindItem<tag, Vector<Others...>> { };

/// Специализация FindItemBool: результат найден, запоминаем типы вектора и значения
template<Enum tag, class ItemT, class... Others>
struct FindItemBool<true, tag, Vector<ItemT, Others...>>
{
    typedef Vector<ItemT, Others...> Vector;
    typedef typename ItemT::Item Item;
};


/// Специализация FindItem: сравниваем метки, продолжаем с помощью FindItemBool
template<Enum tag, class ItemT, class... Others>
struct FindItem<tag, Vector<ItemT, Others...>> :
        public FindItemBool<tag == ItemT::tag, tag, Vector<ItemT, Others...>> { };

/// Функция получения постоянного значения
template<Enum tag, class... Types> inline
constexpr const typename FindItem<tag, Vector<Types...>>::Item & get(const Vector<Types...> & vector)
{
    typedef typename FindItem<tag, Vector<Types...>>::Vector Vector;
    return (((Vector&) vector).getItem());
}

/// Класс, представяющий собой объединение двух векторов
template<class Vector1, class Vector2>
class VectorMerge;

template<bool has, class Vector1, class Vector2>
class VectorMergeBool;

/// Если Type2 входит в Vector1 - просто убираем его
template<class Vector1, typename Type2, typename... Types2>
class VectorMergeBool<true, Vector1, Vector<Type2, Types2...>> : public VectorMerge<Vector1, Vector<Types2...>> { };

template<class Vector, typename... Types> struct VectorPush;

template<typename Type, typename... Types>
struct VectorPush<Vector<Types...>, Type> : Vector<Type, Types...> {};

/// Если Type2 не входит в Vector1 - перекладываем
template<class Vector1, typename Type2, typename... Types2>
class VectorMergeBool<false, Vector1, Vector<Type2, Types2...>> : public VectorMerge<VectorPush<Vector1, Type2>, Vector<Types2...>> { };

template<class Vector1, typename Type2, typename... Types2>
class VectorMerge<Vector1, Vector<Type2, Types2...>> : public VectorMergeBool<Vector1::template has<(Enum)Type2::tag>(), Vector1, Vector<Type2, Types2...>> { };

template<class Vector1>
class VectorMerge<Vector1, Vector<>> : public Vector1 { };

//-------------------------- get ------------------------------------------------------------------//
/// Функция получения значения
template<Enum tag, class... Types> inline
constexpr typename FindItem<tag, Vector<Types...>>::Item & get(Vector<Types...> & vector)
{
    typedef typename FindItem<tag, Vector<Types...>>::Vector Vector;
    return (((Vector&) vector).item.item);
}

/// Рекурсивная специализация вектора
template<class Item, typename... Others>
class Vector<Item, Others...> : private Vector<Others...>
{
    //inline
public:
    typedef Vector<Others...> Parent;
    typedef Vector<Item, Others...> This;
    Item item;
    constexpr typename Item::Item &getItem() { return item.item; }

    template<Enum tag> static constexpr bool has() { return (Enum) Item::tag == tag ? true : Parent::template has<tag>(); }
    template<class Other> static constexpr bool subsetOf() { return Other::template has<(Enum)Item::tag>() ? Parent::template subsetOf<Other>() : false; }
    template<class Other> static constexpr bool subsetOf(const Other &) { return This::template subsetOf<Other>(); }
    //template<Enum tag> constexpr Item &get() { return

    /*template<Enum tag, class... Types> inline
    constexpr const typename FindItem<tag, Vector<Types...>>::Item & get() const
    {
        typedef typename FindItem<tag, Vector<Types...>>::Vector Vector;
        return (((Vector&) *this).item);
    }*/

    template<class Other>
    Vector<Item, Others...> & operator += (const Other & other)
    {
        //static_assert(Other::template subsetOf<This>(), "Unable to add");
        Vector<Others...>::operator +=(other);
        if(other.has<(Enum)Item::tag>())
            item.item += get<Enum(Item::tag)>(other);
        return *this;
    }
    template<class Other>
    inline VectorMerge<This, Other> operator + (const Other & other) const
    {
        VectorMerge<This, Other> result;

        //if()
        return result;
    }
};

template<typename... Vectors>
class Matrix : public Vector<Vectors...>
{
public:
    template<class Other>
    Matrix<Vectors...> operator * (const Other & other) const
    {

    }
};

}

#endif // VECTPL_H
