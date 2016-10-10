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
protected:
    template<class Other> inline void add(const Other &) { }
    template<class Other1, class Other2> inline void sum(const Other1 &, const Other2 &) { }
public:
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
    typedef Vector<> _Vector;
    typedef int Item;
};

/// Специализация FindItemBool: результат не найден, продолжаем поиск в остатке вектора
template<Enum tag, class ItemT, class... Others>
struct FindItemBool<false, tag, Vector<ItemT, Others...>> : public FindItem<tag, Vector<Others...>> { };

/// Специализация FindItemBool: результат найден, запоминаем типы вектора и значения
template<Enum tag, class ItemT, class... Others>
struct FindItemBool<true, tag, Vector<ItemT, Others...>>
{
    typedef Vector<ItemT, Others...> _Vector;
    typedef typename ItemT::Item Item;
};


/// Специализация FindItem: сравниваем метки, продолжаем с помощью FindItemBool
template<Enum tag, class ItemT, class... Others>
struct FindItem<tag, Vector<ItemT, Others...>> :
        public FindItemBool<tag == ItemT::tag, tag, Vector<ItemT, Others...>> { };

/// Класс, выполняющий объединение двух векторов
template<class Vector1, class Vector2>
struct VectorUnion;

template<bool has, class Vector1, class Vector2>
struct VectorUnionBool;

/// Если Type2 входит в Vector1 - просто убираем его
template<class Vector1, typename Type2, typename... Types2>
struct VectorUnionBool<true, Vector1, Vector<Type2, Types2...>> : public VectorUnion<Vector1, Vector<Types2...>> { };

template<class Vector, typename... Types> struct VectorPush;

template<typename Type, typename... Types>
struct VectorPush<Vector<Types...>, Type> { typedef Vector<Type, Types...> Result; };

/// Если Type2 не входит в Vector1 - перекладываем
template<class Vector1, typename Type2, typename... Types2>
class VectorUnionBool<false, Vector1, Vector<Type2, Types2...>> : public VectorUnion<typename VectorPush<Vector1, Type2>::Result, Vector<Types2...>> { };

template<class Vector1, typename Type2, typename... Types2>
struct VectorUnion<Vector1, Vector<Type2, Types2...>> : public VectorUnionBool<Vector1::template has<(Enum)Type2::tag>(), Vector1, Vector<Type2, Types2...>> { };

template<class Vector1>
class VectorUnion<Vector1, Vector<>> : public Vector1 { public: typedef Vector1 Result; };

//-------------------------- get ------------------------------------------------------------------//
/// Функция получения значения
template<Enum tag, class... Types> inline
constexpr typename FindItem<tag, Vector<Types...>>::Item & get(Vector<Types...> & vector)
{
    typedef typename FindItem<tag, Vector<Types...>>::_Vector Vector;
    return (((Vector&) vector).getItem());
}

/// Функция получения постоянного значения
template<Enum tag, class... Types> inline
constexpr const typename FindItem<tag, Vector<Types...>>::Item & get(const Vector<Types...> & vector)
{
    typedef typename FindItem<tag, Vector<Types...>>::_Vector Vector;
    return (((Vector&) vector).getItem());
}


/// Рекурсивная специализация вектора
template<class Item, typename... Others>
class Vector<Item, Others...> : public Vector<Others...>
{
protected:
    template<class Other> inline void add(const Other & other)
    {
        Parent::add(other);
        if(other.has<(Enum)Item::tag>())
            item.item += get<Enum(Item::tag)>(other);
    }
public:
    template<class Other1, class Other2> inline void sum(const Other1 & o1, const Other2 & o2)
    {
        Parent::sum(o1, o2);
        const Enum e = (Enum) Item::tag;
        static_assert(o1.has<e>() || o2.has<e>(), "Nothing to sum()");
        if(o1.has<e>() && o2.has<e>()) item.item = get<e>(o1) + get<e>(o2);
        else if(o1.has<e>()) item.item = get<e>(o1);
        else item.item = get<e>(o2);
    }
    typedef Vector<Others...> Parent;
    typedef Vector<Item, Others...> This;
    Item item;
    constexpr typename Item::Item &getItem()
    {
        return item.item;
    }
    constexpr const typename Item::Item &getItem() const
    {
        return item.item;
    }

    template<Enum tag> static constexpr bool has() { return (Enum) Item::tag == tag ? true : Parent::template has<tag>(); }
    template<class Other> static constexpr bool subsetOf() { return Other::template has<(Enum)Item::tag>() ? Parent::template subsetOf<Other>() : false; }
    template<class Other> static constexpr bool subsetOf(const Other &) { return This::template subsetOf<Other>(); }

    template<Enum tag, class... Types> inline
    constexpr const typename FindItem<tag, Vector<Types...>>::Item & _get() const
    {
        typedef typename FindItem<tag, Vector<Types...>>::Vector Vector;
        const Vector * t = this;
        return t->item.item;
    }

    template<class Other>
    Vector<Item, Others...> & operator += (const Other & other)
    {
        static_assert(Other::template subsetOf<This>(), "Unable to add");
        add(other);
        return *this;
    }
    template<class Other>
    Vector<Item, Others...> & operator = (const Other & other)
    {


    }
    template<class Other>
    inline typename VectorUnion<This, Other>::Result operator + (const Other & other) const
    {
        typename VectorUnion<This, Other>::Result result;
        result.sum(*this, other);
        return result;
    }
};

template<typename... Vectors> class Matrix;
template<typename... Vectors> class MatrixDef;

template<typename... Vectors> struct MatrixDef<Vector<Vectors...>>
{
    typedef Matrix<Vectors...> Result;
};

/// Класс, выполняющий объединение двух матриц
template<class Matrix1, class Matrix2> struct MatrixUnion
{
    typedef typename MatrixDef<typename VectorUnion<typename Matrix1::Parent, typename Matrix2::Parent>::Result>::Result Result;
};

template<typename... Vectors>
class Matrix : public Vector<Vectors...>
{
public:
    typedef Vector<Vectors...> Parent;
    typedef Matrix<Vectors...> This;
    template<class Other>
    inline typename MatrixUnion<This, Other>::Result operator + (const Other & other) const
    {
        typename MatrixUnion<This, Other>::Result result;
        result.sum((const Parent &)*this, (const typename Other::Parent &)other);
        return result;
    }
    template<class Other>
    Matrix<Vectors...> operator * (const Other & other) const
    {
        Matrix<Vectors...> result;

        return result;
    }
};

/// Функция получения значения из матрицы

template<Enum tag1, Enum tag2, class... Vectors> inline
constexpr auto & get(Matrix<Vectors...> & matrix)
{
    auto &vector = get<tag1>((typename Matrix<Vectors...>::Parent &)matrix);
    return get<tag2>(vector);
}

template<Enum tag1, Enum tag2, class... Vectors> inline
constexpr const auto & get(const Matrix<Vectors...> & matrix)
{
    const auto &vector = get<tag1>((const typename Matrix<Vectors...>::Parent &)matrix);
    return get<tag2>(vector);
}


}

#endif // VECTPL_H
