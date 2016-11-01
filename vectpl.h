#ifndef VECTPL_H
#define VECTPL_H
#include <assert.h>

namespace vtp {

struct None { };

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
    static constexpr size_t size() { return 0; }
public:
    //constexpr bool has(Enum) const { return false; }
    template<Enum tag> static constexpr bool has() { return false; }
    template<class Other> static constexpr bool subsetOf() { return true; }
    constexpr None &getItem() const { return *(None *)nullptr; }
    //template<Enum tag> constexpr get
};

//-------------------------- Reverse -------------------------------------------------------------------//

template<class Vector1, class Vector2> struct ToLeft;

template<typename... Types1, typename Type2, typename... Types2>
struct ToLeft<Vector<Types1...>, Vector<Type2, Types2...>> : public ToLeft<Vector<Type2, Types1...>, Vector<Types2...>> { };

template<typename... Types>
struct ToLeft<Vector<Types...>, Vector<>> { typedef Vector<Types...> Result; };

template<class Src>
using Reverse = typename ToLeft<Vector<>, Src>::Result;

//-------------------------- MaybePush ----------------------------------------------------------------//

template<class Dst, class Item> struct MaybePushT;
template<template<class...> class VectorT, typename... Dst> struct MaybePushT<VectorT<Dst...>, None>
{
    typedef VectorT<Dst...> Result;
};

template<template<class...> class VectorT, typename... Dst, typename Item> struct MaybePushT<VectorT<Dst...>, Item>
{
    typedef VectorT<Dst..., Item> Result;
};

template<class Dst, class Item>
using MaybePush = typename MaybePushT<Dst, Item>::Result;

//-------------------------- Map ----------------------------------------------------------------------//

template<class Dst, class Src, template<class...> class Transform, class... Params> struct MapT;

template<template<class...> class VectorT, typename... Dst, typename Item, typename... Src, template<class...> class Transform, class... Params>
struct MapT<
        VectorT<Dst...>,
        VectorT<Item, Src...>,
        Transform, Params...
    > : public MapT<
        MaybePush<VectorT<Dst...>, Transform<Item, Params...>>,// VectorT<Dst..., Transform<Item, Params...>>,
        VectorT<Src...>,
        Transform, Params...
    > {};

template<template<class...> class VectorT, typename... Dst, template<class...> class Transform, class... Params>
struct MapT<
        VectorT<Dst...>,
        VectorT<>,
        Transform,
        Params...
    > { typedef VectorT<Dst...> Result; };

template<class Src, template<class...> class Transform, class... Params> struct MapTHelper;

template<template<class...> class VectorT, typename... Src, template<class...> class Transform, class... Params>
struct MapTHelper<
        VectorT<Src...>,
        Transform,
        Params...
    > : public MapT<
        VectorT<>,
        VectorT<Src...>,
        Transform,
        Params...
    > {};

template<class VectorT, template<class...> class Transform, class... Params>
using Map = typename MapTHelper<VectorT, Transform, Params...>::Result;

//-------------------------- Fold ----------------------------------------------------------------------//
template<class Dst, class Src, template<class...> class Transform, class... Params> struct FoldT;

template<typename Dst, template<class...> class VectorT, typename Item, typename... Src, template<class...> class Transform, class... Params>
struct FoldT<
        Dst,
        VectorT<Item, Src...>,
        Transform, Params...
    > : public FoldT<
        Transform<Dst, Item, Params...>,
        VectorT<Src...>,
        Transform, Params...
    > {};

template<template<class...> class VectorT, typename Dst, template<class...> class Transform, class... Params>
struct FoldT<
        Dst,
        VectorT<>,
        Transform,
        Params...
    > { typedef Dst Result; };

template<class Init, class VectorT, template<class...> class Transform, class... Params>
using Fold = typename FoldT<Init, VectorT, Transform, Params...>::Result;

//-------------------------- Find ----------------------------------------------------------------------------//

template<typename Result, typename Array, template<class...> class Transform, class... Params> struct FindT;

template<template<class...> class VectorT, typename Item, typename... Src, template<class...> class Transform, class... Params>
struct FindT<
        None,
        VectorT<Item, Src...>,
        Transform, Params...
    > : public FindT<
        Transform<Item, Params...>,
        VectorT<Src...>,
        Transform, Params...
    > { };

template<template<class...> class VectorT, typename Item, typename... Src, template<class...> class Transform, class... Params>
struct FindT<
        Item,
        VectorT<Src...>,
        Transform, Params...
    > { typedef Item Result; };

template<class Array, template<class...> class Transform, class... Params>
using Find = typename FindT<None, Array, Transform, Params...>::Result;

//--------------------------- GetContainerType ---------------------------------------------------------------//

template<class Container> struct GetContainerTypeT;

template<template<class...> class ContainerType, class... Items>
struct GetContainerTypeT<ContainerType<Items...>> { template<class... Is> using Result = ContainerType<Is...>; };

template<class Container>
using GetContainerType = typename GetContainerTypeT<Container>::Result;

//-------------------------- InnerJoin ------------------------------------------------------------//

template<class V1, class V2, template<class...> class Join, class... Params>
struct InnerJoinT
{
    template<class Item1, class Item2> struct FinderT { typedef None Result; };

    template<Enum tag, class Item1, class Item2> struct FinderT<Tag<tag, Item1>, Tag<tag, Item2>>
    {
        typedef Tag<tag, Join<Item1, Item2, Params...>> Result;
    };

    template<class Item1, class Item2> using Finder = typename FinderT<Item1, Item2>::Result;

    template<class Item>
    using JFind = Find<V2, Finder, Item>;

    using Result = Map<V1, JFind>;
};

template<class V1, class V2, template<class...> class Join, class... Params>
using InnerJoin = typename InnerJoinT<V1, V2, Join, Params...>::Result;

//-------------------------- MakeVectorTranspose -------------------------------------------------------------//

template<class Item, class Param>
using MakeItemTranspose = Tag<
    (Enum)Item::tag,
    Vector<Tag<(Enum)Param::tag, typename Item::Item>>
>;

template<class VectorT, Enum tag>
using MakeVectorTranspose = Map<VectorT, MakeItemTranspose, Tag<tag, None>>;

//-------------------------- FindItem ------------------------------------------------------------------//

/// Шаблон для поиска элемента вектора по метке
template<Enum tag, class Item> struct FindItemT;

/// Шаблон FindItem с указанием найдена метка или нет
template<bool found, Enum tag, class Item> struct FindItemBool;

/// Специализация FindItem: ничего не найдено
template<Enum tag>
struct FindItemT<tag, Vector<>>
{
    typedef Vector<> _Vector;
    typedef None Item;
};

/// Специализация FindItemBool: результат не найден, продолжаем поиск в остатке вектора
template<Enum tag, class ItemT, class... Others>
struct FindItemBool<false, tag, Vector<ItemT, Others...>> : public FindItemT<tag, Vector<Others...>> { };

/// Специализация FindItemBool: результат найден, запоминаем типы вектора и значения
template<Enum tag, class ItemT, class... Others>
struct FindItemBool<true, tag, Vector<ItemT, Others...>>
{
    typedef Vector<ItemT, Others...> _Vector;
    typedef typename ItemT::Item Item;
};


/// Специализация FindItem: сравниваем метки, продолжаем с помощью FindItemBool
template<Enum tag, class ItemT, class... Others>
struct FindItemT<tag, Vector<ItemT, Others...>> :
        public FindItemBool<tag == ItemT::tag, tag, Vector<ItemT, Others...>> { };


#define FindItem(tag, vec) typename FindItemT<(Enum)tag, vec>::Item

//-------------------------- Union ------------------------------------------------------------------//

/// Класс, выполняющий объединение
template<typename Class1, typename Class2>
struct UnionT;

template<typename Class1, typename Class2>
using Union = typename UnionT<Class1, Class2>::Result;
#define TagUnion(t, a, b) Tag<(Enum)t, typename UnionT<a, b>::Result>

//template<typename T> struct Union<T, T> { typedef T Result; };

template<> struct UnionT<double, double> { typedef double Result; };
template<> struct UnionT<int, int> { typedef int Result; };
template<class T> struct UnionT<T, None> { typedef T Result; };
template<class T> struct UnionT<None, T> { typedef T Result; };

template<class Vector, typename... Types> struct VectorPush;

template<typename Type, typename... Types>
struct VectorPush<Vector<Types...>, Type> { typedef Vector<Type, Types...> Result; };

template<typename... Types>
struct VectorPush<Vector<Types...>, None> { typedef Vector<Types...> Result; };

template<class Vector, typename Type, typename Caution> struct VectorPushIfNone;

template<typename Type, typename... Types>
struct VectorPushIfNone<Vector<Types...>, Type, None> { typedef Vector<Types..., Type> Result; };

template<typename Type, typename... Types, typename Caution>
struct VectorPushIfNone<Vector<Types...>, Type, Caution> { typedef Vector<Types...> Result; };

template<class DstVector, class SrcVector, class MergeVector> struct VectorMergeLeft;

template<typename... Dst, typename Type, typename... Src, typename Mrg>
struct VectorMergeLeft<
            Vector<Dst...>,
            Vector<Type, Src...>,
            Mrg
        > : public VectorMergeLeft<
            Vector<Dst..., TagUnion(Type::tag, typename Type::Item, FindItem(Type::tag, Mrg))>,
            Vector<Src...>,
            Mrg
        > {};

template<typename... Dst, typename Mrg>
struct VectorMergeLeft<Vector<Dst...>, Vector<>, Mrg>
{
    typedef Vector<Dst...> Result;
};

template<class DstVector, class SrcVector> struct VectorAbsentAdd;

template<typename... Dst, typename Type, typename... Src>
struct VectorAbsentAdd<
            Vector<Dst...>,
            Vector<Type, Src...>
        > : public VectorAbsentAdd<
            typename VectorPushIfNone<Vector<Dst...>, Type, FindItem(Type::tag, Vector<Dst...>)>::Result,
            Vector<Src...>
        > {};

template<typename... Dst>
struct VectorAbsentAdd<Vector<Dst...>, Vector<>>
{
    typedef Vector<Dst...> Result;
};

template<typename... Types1, typename... Types2>
struct UnionT<Vector<Types1...>, Vector<Types2...>>
{
    typedef typename VectorMergeLeft<
        Vector<>,
        Vector<Types1...>,
        Vector<Types2...>
    >::Result MergeResult;
    typedef typename VectorAbsentAdd<
        MergeResult,
        Vector<Types2...>
    >::Result Result;
};

template<class Vector1>
class UnionT<Vector1, Vector<>> : public Vector1 { public: typedef Vector1 Result; };

//-------------------------- get ------------------------------------------------------------------//
/// Функция получения значения
template<Enum tag, class... Types> inline
constexpr typename FindItemT<tag, Vector<Types...>>::Item & get(Vector<Types...> & vector)
{
    typedef typename FindItemT<tag, Vector<Types...>>::_Vector Vector;
    return (((Vector&) vector).getItem());
}

/// Функция получения постоянного значения
template<Enum tag, class... Types> inline
constexpr const typename FindItemT<tag, Vector<Types...>>::Item & get(const Vector<Types...> & vector)
{
    typedef typename FindItemT<tag, Vector<Types...>>::_Vector Vector;
    return (((Vector&) vector).getItem());
}


/// Рекурсивная специализация вектора
template<class Item, typename... Others>
class Vector<Item, Others...> : public Vector<Others...>
{
protected:
    inline void itemAdd(const None &) { }
    template<class Value> inline void itemAdd(const Value & value) { item.item += value; }
    template<class Other> inline void add(const Other & other)
    {
        Parent::add(other);
        itemAdd(get<Enum(Item::tag)>(other));
    }
    template<class V1, class V2> inline void itemSum(const V1 & v1, const V2 & v2) { item.item = v1 + v2; }
    template<class V1> inline void itemSum(const V1 & v1, const None &) { item.item = v1; }
    template<class V2> inline void itemSum(const None &, const V2 & v2) { item.item = v2; }
    template<class Other> inline void assign(const Other & other)
    {
        Parent::assign(other);
        if(other.has<(Enum)Item::tag>())
            item.item = get<Enum(Item::tag)>(other);
        else
            item.item = 0;
    }
public:
    static constexpr size_t size() { return Parent::size() + 1; }
    template<class Other1, class Other2> inline void sum(const Other1 & o1, const Other2 & o2)
    {
        Parent::sum(o1, o2);
        const Enum e = (Enum) Item::tag;
        static_assert(o1.has<e>() || o2.has<e>(), "Nothing to sum()");
        itemSum(get<e>(o1), get<e>(o2));
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
    constexpr const typename FindItemT<tag, Vector<Types...>>::Item & _get() const
    {
        typedef typename FindItemT<tag, Vector<Types...>>::Vector Vector;
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
        static_assert(Other::template subsetOf<This>(), "Unable to assign");
        assign(other);
        return *this;
    }
    template<class Other>
    inline typename UnionT<This, Other>::Result operator + (const Other & other) const
    {
        typename UnionT<This, Other>::Result result;
        result.sum(*this, other);
        return result;
    }
};

template<typename... Vectors> class Matrix;
template<typename... Vectors> class MatrixDef;
template<class Dst, class Src> struct MakeMatrixTransposeT;

template<typename... Vectors> struct MatrixDef<Vector<Vectors...>>
{
    typedef Matrix<Vectors...> Result;
};

/// Класс, выполняющий объединение двух матриц
template<class... Vectors1, class Matrix2> struct UnionT<Matrix<Vectors1...>, Matrix2>
{
    typedef typename MatrixDef<typename UnionT<typename Matrix<Vectors1...>::Vectors, typename Matrix2::Vectors>::Result>::Result Result;
};

template<typename... VectorsT>
class Matrix : public Vector<VectorsT...>
{
public:
    typedef Vector<VectorsT...> Vectors;
    typedef Matrix<VectorsT...> This;
    template<class Other>
    inline typename UnionT<This, Other>::Result operator + (const Other & other) const
    {
        typename UnionT<This, Other>::Result result;
        result.sum((const Vectors &)*this, (const typename Other::Vectors &)other);
        return result;
    }
    template<class Other>
    Matrix<VectorsT...> operator * (const Other & other) const
    {
        Matrix<VectorsT...> result;

        return result;
    }
};

/// Функция получения значения из матрицы

template<Enum tag1, Enum tag2, class... VectorsT> inline
constexpr auto & get(Matrix<VectorsT...> & matrix)
{
    auto &vector = get<tag1>((typename Matrix<VectorsT...>::Vectors &)matrix);
    return get<tag2>(vector);
}

template<Enum tag1, Enum tag2, class... VectorsT> inline
constexpr const auto & get(const Matrix<VectorsT...> & matrix)
{
    const auto &vector = get<tag1>((const typename Matrix<VectorsT...>::Vectors &)matrix);
    return get<tag2>(vector);
}

//-------------------------- MakeMatrixTranspose -------------------------------------------------------------//

template<class DstMatrix, typename Item, class... Src>
struct MakeMatrixTransposeT<
            DstMatrix,
            Matrix<Item, Src...>
        > : public MakeMatrixTransposeT<
            typename UnionT<
                DstMatrix,
                typename MatrixDef<MakeVectorTranspose<typename Item::Item, (Enum)Item::tag>>::Result
            >::Result,
            Matrix<Src...>
        > {};

template<class DstMatrix>
struct MakeMatrixTransposeT<DstMatrix, Matrix<>>
{
    typedef DstMatrix Result;
};

template<class MatrixT>
using MatrixTranspose = typename MakeMatrixTransposeT<Matrix<>, MatrixT>::Result;

//------------------------- Mul --------------------------------------

template<class A, class B>
using Sum = Union<A, B>;

template<class A, class B>
using Mul = Union<A, B>;

template<class V1, class V2>
using VecVecMul = Fold<None, InnerJoin<V1, V2, Mul>, Sum>; // Делаем сложение

/*template<class V1, class M2T>
using VecMatMul = Tag<V1::tag, Map<M2T, VecVecMul, typename V1::Item>>; // Произведение вектора на матрицу

template<class M1, class M2>
using MatrixMul = Map<M1, VecMatMul, MatrixTranspose<M2>>; // Произведение матриц
*/

}

/* Regexp
 *  vtp::Tag<\(Enum\)(\d)u, (\l+)>
 *  $1:$2
 *
 *  vtp::Tag<\(Enum\)(\d)u, vtp::Vector<([\w:, ]+)> *>
 *  $1:($2)
 *
 *  vtp::Vector<([\w:\(, \)]+)>
 *  $1
 */


#endif // VECTPL_H
