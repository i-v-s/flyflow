#ifndef VECTPL_H
#define VECTPL_H

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
    template<Enum tag> constexpr bool has() const { return false; }
    constexpr int &getItem() const { return *(int *) nullptr; }
    //template<Enum tag> constexpr get
};

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

/// Функция получения значения
template<Enum tag, class... Types> inline
constexpr typename FindItem<tag, Vector<Types...>>::Item & get(Vector<Types...> & vector)
{
    typedef typename FindItem<tag, Vector<Types...>>::Vector Vector;
    return (((Vector&) vector).item.item);
}

/// Рекурсивная специализация вектора
template<class Item, typename... Other>
class Vector<Item, Other...> : private Vector<Other...>
{
public:
    typedef Vector<Other...> Parent;
    Item item;
    constexpr typename Item::Item &getItem() { return item.item; }

    //constexpr bool has(Enum tag) const { return (Enum) Item::tag == tag ? true : Parent::has(tag); }
    template<Enum tag> constexpr bool has() const { return (Enum) Item::tag == tag ? true : Parent::template has<tag>(); }
    //template<Enum tag> constexpr Item &get() { return

    /*template<Enum tag, class... Types> inline
    constexpr const typename FindItem<tag, Vector<Types...>>::Item & get() const
    {
        typedef typename FindItem<tag, Vector<Types...>>::Vector Vector;
        return (((Vector&) *this).item);
    }*/

    template<class O>
    Vector<Item, Other...> & operator += (const O & other)
    {
        Vector<Other...>::operator +=(other);
        if(other.has<(Enum)Item::tag>())
            item.item += get<Enum(Item::tag)>(other);
        return *this;
    }
};

/*struct Matrix
{
};

template<Enum A, Enum B>
int &get(Matrix &mat) { }

template<>
int &get<Attitude, Velocity>(Matrix &mat) { }
*/

}

#endif // VECTPL_H
