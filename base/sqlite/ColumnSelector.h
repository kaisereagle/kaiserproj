#pragma once
/**
 * @file      ColumnSelector.h
 * @brief     テーブル構造体の配列から高速に検索するためのクラス
 * @usage
 *            ～～～基本的な使い方～～～
 *            
 *            1.最初に一回インスタンスを作成
 *            ColumnSelector<master_unit, int64_t> master_unit__prefab_id =
 *                                    new ColumnSelector<master_unit, int64_t>(&ac_master_unit(0), ac_master_unit_count(), &master_unit::prefab_id);
 *            
 *            2.1件検索する
 *            if(auto record = master_unit__prefab_id->find(prefab_id))
 *            {}
 *
 *            3.複数件検索し、ループ処理する
 *            for(auto record : master_unit__prefab_id->findRange(prefab_id))
 *            {}
 *            
 *            ～～～応用的な使い方～～～
 *            
 *            1.複数のカラムで検索したい場合
 *            ColumnSelector<master_unit, int64_t, const char*> master_unit__prefab_id__name =
 *                                    new ColumnSelector<master_unit, int64_t, const char*>(&ac_master_unit(0), ac_master_unit_count(), &master_unit::prefab_id, &master_unit::name);
 *
 *            2.全てのカラムの値を指定して検索
 *            master_unit__prefab_id__name->find(prefab_id, name);
 *
 *            3.複数カラムでセットアップした場合は途中のカラムまでの指定も可能
 *            master_unit__prefab_id__name->find(prefab_id);
 *
 *            4.検索対象をフィルタリングして初期化したい場合
 *            ColumnSelector<master_unit, int64_t> master_unit__prefab_id__rare4 =
 *                                    new ColumnSelector<master_unit, int64_t>(&ac_master_unit(0), ac_master_unit_count(), [](const master_unit* data)
 *			                          {
 *                                        return data->rare == 4;		// レアリティが4のものだけ対象とする
 *                                    }, &master_unit::prefab_id);
 *
 *            ～～～範囲オブジェクト(Range/ReverseRange)の使い方～～～
 *
 *            1.範囲の数を取得したい場合
 *            auto range = selector->findRange(column_value);
 *            auto num = range.size();									// size()関数で取得
 *            for(auto record : range)table[--num] = record->name;
 *
 *            2.範囲を降順で列挙したい場合
 *            for(auto record : selector->findRange(column_value).reverse())
 *            {}
 *
 *            3.範囲の取得数に制限を設けたい場合
 *            for(auto record : selector->findRange(column_value).limit(10))	// 最初の10件だけ列挙
 *            {}
 *
 *            4.範囲の開始位置をずらしたい場合
 *            for(auto record : selector->findRange(column_value).skip(10))		// 最初の10件は飛ばす
 *            {}
 *
 *            5.これらの機能を組み合わせる
 *            for(auto record : selector->findRange(column_value).reverse().skip(30).limit(10))	// 降順で30～39番目の要素だけ列挙
 *            {}
 *
 * @author    Shunsuke Kuribara
 * @date      2016-06-14
 * @copyright (C) 2016 Flame Hearts Inc.
 */
#ifndef __COLUMN_SELECTOR__
#define __COLUMN_SELECTOR__

#include <algorithm>
#include <tuple>
#include <assert.h>
#include <string.h>
#include <string>

namespace ray_conf
{
	namespace detail
	{
		template<typename X>
		inline static int compareValue(X a, X b)
		{
			static_assert(std::is_arithmetic<X>::value, "型が数値型ではありません");
			const auto sub = static_cast<typename std::make_signed<X>::type>(a - b);
			return sub < 0 ? -1 : sub > 0 ? 1 : 0;
		}

		inline static int compareValue(const char* a, const char* b)
		{
			return strcmp(a, b);
		}

		inline static int compareValue(const std::string& a, const std::string& b)
		{
			return a.compare(b);
		}

		template<typename T, typename S>
		struct MemberType
		{
			typedef S(T::*type);
		};

		template<typename T, class S>
		inline static volatile const void* offset_of(S(T::*pm))
		{
			return reinterpret_cast<volatile const void*>(
				&reinterpret_cast<const volatile char&>(((T*)0)->*pm)
				);
		}

		template<typename T, typename Members, size_t ArgNum, size_t i, bool Last = ArgNum == i>
		struct Helper
		{
			typedef const T* Element;
			template<typename S>
			inline static bool contains(const Members& members, S(T::*member))
			{
                if (offset_of(std::get<i>(members)) == offset_of(member))
				{
					return true;
				}
				return Helper<T, Members, ArgNum, i + 1>::contains(members, member);
			}

			inline static int compare(const Members& members, Element a, Element b)
			{
				auto member = std::get<i>(members);
				auto value = compareValue(a->*member, b->*member);
				if (value != 0)
				{
					return value;
				}
				return Helper<T, Members, ArgNum, i + 1>::compare(members, a, b);
			}
		};

		template<typename T, typename Members, size_t ArgNum, size_t i>
		struct Helper<T, Members, ArgNum, i, true>
		{
			typedef const T* Element;
			template<typename S>
			inline static bool contains(const Members&, S(T::*))
			{
				return false;
			}

			inline static int compare(const Members&, Element, Element)
			{
				return 0;
			}
		};

		template<size_t i, typename T, typename Members, typename V>
		inline void setMember(const Members& members, T& tmp, const V& value)
		{
			tmp.*std::get<i>(members) = value;
		}

		template<size_t i, typename T, typename Members, typename V, typename ...S>
		inline void setMember(const Members& members, T& tmp, const V& value, const S&...others)
		{
			tmp.*std::get<i>(members) = value;
			setMember<i + 1>(members, tmp, others...);
		}
	}
}

// T:テーブルの型
// U:カラムの型
template<typename T, typename...U>
/**
 * あるテーブルの任意のカラムで高速に検索（バイナリサーチ）するためのユーティリティ
 * 検索かけたいカラムの数だけインスタンスを作る必要あり
 */
class ColumnSelector
{
	typedef const T* Element;
	// テーブル要素のアドレスを格納する配列
	Element* begin;
	Element* end;
	typedef std::tuple<typename ray_conf::detail::MemberType<T, U>::type...> Members;

	Members members;

	template<size_t Limit = sizeof...(U)>
	struct Helper : ray_conf::detail::Helper<T, Members, Limit, 0>
	{
	};

public:

	/**
	 * 指定カラムでソート済み配列を作成
	 * @param data テーブルの先頭ポインタ
	 * @param num レコード数
	 * @param member ソートしたいカラムを優先順位順に渡す（テーブルのメンバー変数ポインタ）
	 */
	ColumnSelector(const T* data, uint32_t num, typename ray_conf::detail::MemberType<T, U>::type...member)
		:begin(new Element[num])
		, end(begin + num)
		, members(member...)
	{
		for (uint32_t i = 0; i < num; ++i)
		{
			begin[i] = data++;
		}
		std::stable_sort(begin, end, [this](Element a, Element b)
		{
			return Helper<>::compare(members, a, b) < 0;
		});
	}

	template<typename Filter>
	/**
	 * 指定カラムでソート済み配列を作成
	 * @param data テーブルの先頭ポインタ
	 * @param num レコード数
	 * @param filter カラム要素をフィルタリングする関数 [bool filter(const T*)]
	 * @param member ソートしたいカラムを優先順位順に渡す（テーブルのメンバー変数ポインタ）
	 */
	ColumnSelector(const T* data, uint32_t num, const Filter& filter, typename ray_conf::detail::MemberType<T, U>::type...member)
		:begin(new Element[num])
		, end(begin + num)
		, members(member...)
	{
		uint32_t count = 0;
		for (uint32_t i = 0; i < num; ++i)
		{
			if (filter(data + i))
			{
				begin[count++] = data + i;
			}
		}
		// 領域再確保
		if (count != num)
		{
			auto tmp = new Element[count];
			memcpy(tmp, begin, sizeof(Element) * count);
			delete[] begin;
			begin = tmp;
			end = tmp + count;
		}

		std::stable_sort(begin, end, [this](Element a, Element b)
		{
			return Helper<>::compare(members, a, b) < 0;
		});
	}

	// ムーブ
	ColumnSelector(ColumnSelector&& value)
		:begin(value.begin)
		, end(value.end)
		, members(value.members)
	{
		value.begin = nullptr;
		//value.end = nullptr;
		//value.member = nullptr;
	}

	// ムーブ
	ColumnSelector& operator=(ColumnSelector&& value)
	{
		delete[] begin;
		begin = value.begin;
		end = value.end;
		members = value.members;
		value.begin = nullptr;
		//value.end = nullptr;
		//value.member = nullptr;
	}

	~ColumnSelector()
	{
		delete[] begin;
	}

	uint32_t size() const
	{
		return end - begin;
	}

	// 範囲を逆順に取り出す
	class ReverseRange;

	// 範囲を保持するデータ
	class Range : std::pair<const Element*, const Element*>
	{
		friend class ColumnSelector;
		typedef std::pair<const Element*, const Element*> data_type;
		Range(const data_type& value)
			:data_type(value)
		{
			}

	public:

		inline const Element* begin() const
		{
			return data_type::first;
		}

		inline const Element* end() const
		{
			return data_type::second;
		}

		inline uint32_t size() const
		{
			return end() - begin();
		}

		/**
		 * この範囲の逆順の範囲オブジェクトを取得
		 */
		const ReverseRange reverse() const;

		/**
		 * 上限で範囲を絞ったオブジェクトを取得
		 * numが範囲の持つ要素数を超えている場合はそのままの範囲が返ります
		 */
		inline const Range limit(uint32_t num) const
		{
			return Range(data_type(begin(), (std::min)(begin() + num, end())));
		}

		/**
		 * 最初のnum件を飛ばした範囲オブジェクトを取得
		 * numが範囲の持つ要素数を超えている場合は空の範囲が返ります
		 */
		inline const Range skip(uint32_t num) const
		{
			return Range(data_type((std::min)(begin() + num, end()), end()));
		}
	};

	class ReverseRange : std::pair<const Element*, const Element*>
	{
		typedef std::pair<const Element*, const Element*> data_type;
		friend class ColumnSelector::Range;
		ReverseRange(const data_type& value)
			:data_type(value)
		{
		}

	public:

		inline std::reverse_iterator<const Element*> begin() const
		{
			return std::reverse_iterator<const Element*>(data_type::second);
		}

		inline std::reverse_iterator<const Element*> end() const
		{
			return std::reverse_iterator<const Element*>(data_type::first);
		}

		inline uint32_t size() const
		{
			return data_type::second - data_type::first;
		}

		/**
		 * 上限で範囲を絞ったオブジェクトを取得
		 * numが範囲の持つ要素数を超えている場合はそのままの範囲が返ります
		 */
		inline const ReverseRange limit(uint32_t num) const
		{
			return ReverseRange(data_type((std::max)(data_type::second - num, data_type::first), data_type::second));
		}

		/**
		 * 最初のnum件を飛ばした範囲オブジェクトを取得
		 * numが範囲の持つ要素数を超えている場合は空の範囲が返ります
		 */
		inline const ReverseRange skip(uint32_t num) const
		{
			return ReverseRange(data_type(data_type::first, (std::max)(data_type::second - num, data_type::first)));
		}
	};

	template<typename V, typename...S>
	/**
	 * 最初に指定したカラムの値を順に指定してマッチするレコードを検索
	 * 全てのカラムを指定しなくても良い
	 */
	const Element find(const V& value, const S&...values) const
	{
		T tmp;
		ray_conf::detail::setMember<0>(members, tmp, value, values...);

		uint32_t l = 0U;
		uint32_t h = end - begin;
		for (uint32_t i = (h + l) >> 1;;)
		{
			auto evaluate = Helper<sizeof...(S) + 1>::compare(members, &tmp, begin[i]);
			if (evaluate < 0)
			{
				h = i;
				i = (h + l) >> 1;
				if (h == i)break;
			}
			else if (evaluate > 0)
			{
				l = i;
				i = (h + l) >> 1;
				if (l == i)break;
			}
			else
			{
				return begin[i];
			}
		}
		return nullptr;
	}

	template<typename V, typename...S>
	/**
	 * 最初に指定したカラムの値を順に指定してマッチする範囲を検索
	 * 全てのカラムを指定しなくても良い
	 */
	const Range findRange(const V& value, const S&...values) const
	{
		T tmp;
		ray_conf::detail::setMember<0>(members, tmp, value, values...);
		return Range(std::equal_range(begin, end, &tmp, [this](Element a, Element b)
		{
			return Helper<sizeof...(S) + 1>::compare(members, a, b) < 0;
		}));
	}
};

template<typename T, typename ...U>
inline const typename ColumnSelector<T, U...>::ReverseRange ColumnSelector<T, U...>::Range::reverse() const
{
	return ReverseRange(*this);
}

#endif
