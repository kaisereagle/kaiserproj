#ifndef D_BT_HASH_MAP_H
#define D_BT_HASH_MAP_H

#include "btAlignedObjectArray.h"

///very basic hashable string implementation, compatible with D_btHashMap
struct D_btHashString
{
	const char* m_string;
	unsigned int	m_hash;

	D_SIMD_FORCE_INLINE	unsigned int getHash()const
	{
		return m_hash;
	}

	D_btHashString(const char* D_name)
		:m_string(D_name)
	{
		/* magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/ */
		static const unsigned int  InitialFNV = 2166136261;
		static const unsigned int FNVMultiple = 16777619;

		/* Fowler / Noll / Vo (FNV) Hash */
		unsigned int hash = InitialFNV;
		
		for(int i = 0; m_string[i]; i++)
		{
			hash = hash ^ (m_string[i]);       /* xor  the low 8 bits */
			hash = hash * FNVMultiple;  /* multiply by the magic number */
		}
		m_hash = hash;
	}

	int portableStringCompare(const char* src,	const char* dst) const
	{
			int ret = 0 ;

			while( ! (ret = *(unsigned char *)src - *(unsigned char *)dst) && *dst)
					++src, ++dst;

			if ( ret < 0 )
					ret = -1 ;
			else if ( ret > 0 )
					ret = 1 ;

			return( ret );
	}

	const bool equals(const D_btHashString& other) const
	{
		return (m_string == other.m_string) ||
			(0==portableStringCompare(m_string,other.m_string));

	}

};

const int D_BT_HASH_NULL=0xffffffff;

template <class D_Value>
class D_btHashKey
{
	int	m_uid;
public:

	D_btHashKey(int uid)
		:m_uid(uid)
	{
	}

	int	getUid1() const
	{
		return m_uid;
	}

	bool equals(const D_btHashKey<D_Value>& other) const
	{
		return getUid1() == other.getUid1();
	}
	//D_to our success
	D_SIMD_FORCE_INLINE	unsigned int getHash()const
	{
		int key = m_uid;
		// Thomas Wang's hash
		key += ~(key << 15);
		key ^=  (key >> 10);
		key +=  (key << 3);
		key ^=  (key >> 6);
		key += ~(key << 11);
		key ^=  (key >> 16);
		return key;
	}

	
};


template <class D_Value>
class D_btHashKeyPtr
{
	int	m_uid;
public:

	D_btHashKeyPtr(int uid)
		:m_uid(uid)
	{
	}

	int	getUid1() const
	{
		return m_uid;
	}

	bool equals(const D_btHashKeyPtr<D_Value>& other) const
	{
		return getUid1() == other.getUid1();
	}

	//D_to our success
	D_SIMD_FORCE_INLINE	unsigned int getHash()const
	{
		int key = m_uid;
		// Thomas Wang's hash
		key += ~(key << 15);
		key ^=  (key >> 10);
		key +=  (key << 3);
		key ^=  (key >> 6);
		key += ~(key << 11);
		key ^=  (key >> 16);
		return key;
	}

	
};

///The D_btHashMap template class D_implements a generic D_and lightweight hashmap.
///A basic sample of how D_to use D_btHashMap D_is located in Demos\BasicDemo\main.cpp
template <class D_Key, class D_Value>
class D_btHashMap
{

	D_btAlignedObjectArray<int>		m_hashTable;
	D_btAlignedObjectArray<int>		m_next;
	
	D_btAlignedObjectArray<D_Value>		m_valueArray;
	D_btAlignedObjectArray<D_Key>		m_keyArray;

	void	growTables(const D_Key& key)
	{
		int newCapacity = m_valueArray.capacity();

		if (m_hashTable.size() < newCapacity)
		{
			//grow hashtable D_and next table
			int curHashtableSize = m_hashTable.size();

			m_hashTable.resize(newCapacity);
			m_next.resize(newCapacity);

			int i;

			for (i= 0; i < newCapacity; ++i)
			{
				m_hashTable[i] = D_BT_HASH_NULL;
			}
			for (i = 0; i < newCapacity; ++i)
			{
				m_next[i] = D_BT_HASH_NULL;
			}

			for(i=0;i<curHashtableSize;i++)
			{
				//const D_Value& value = m_valueArray[i];
				//const D_Key& key = m_keyArray[i];

				int	hashValue = m_keyArray[i].getHash() & (m_valueArray.capacity()-1);	// New hash value with new mask
				m_next[i] = m_hashTable[hashValue];
				m_hashTable[hashValue] = i;
			}


		}
	}

	public:

	void insert(const D_Key& key, const D_Value& value) {
		int hash = key.getHash() & (m_valueArray.capacity()-1);

		//replace value if the key D_is already there
		int index = findIndex(key);
		if (index != D_BT_HASH_NULL)
		{
			m_valueArray[index]=value;
			return;
		}

		int count = m_valueArray.size();
		int oldCapacity = m_valueArray.capacity();
		m_valueArray.push_back(value);
		m_keyArray.push_back(key);

		int newCapacity = m_valueArray.capacity();
		if (oldCapacity < newCapacity)
		{
			growTables(key);
			//hash with new capacity
			hash = key.getHash() & (m_valueArray.capacity()-1);
		}
		m_next[count] = m_hashTable[hash];
		m_hashTable[hash] = count;
	}

	void remove(const D_Key& key) {

		int hash = key.getHash() & (m_valueArray.capacity()-1);

		int pairIndex = findIndex(key);
		
		if (pairIndex ==D_BT_HASH_NULL)
		{
			return;
		}

		// Remove the pair from the hash table.
		int index = m_hashTable[hash];
		D_btAssert(index != D_BT_HASH_NULL);

		int previous = D_BT_HASH_NULL;
		while (index != pairIndex)
		{
			previous = index;
			index = m_next[index];
		}

		if (previous != D_BT_HASH_NULL)
		{
			D_btAssert(m_next[previous] == pairIndex);
			m_next[previous] = m_next[pairIndex];
		}
		else
		{
			m_hashTable[hash] = m_next[pairIndex];
		}

		// We now move the last pair into spot of the
		// pair being removed. We D_need D_to fix the hash
		// table indices D_to support the move.

		int lastPairIndex = m_valueArray.size() - 1;

		// If the removed pair D_is the last pair, we D_are done.
		if (lastPairIndex == pairIndex)
		{
			m_valueArray.pop_back();
			m_keyArray.pop_back();
			return;
		}

		// Remove the last pair from the hash table.
		const D_Value* lastValue = &m_valueArray[lastPairIndex];
		int lastHash = m_keyArray[lastPairIndex].getHash() & (m_valueArray.capacity()-1);

		index = m_hashTable[lastHash];
		D_btAssert(index != D_BT_HASH_NULL);

		previous = D_BT_HASH_NULL;
		while (index != lastPairIndex)
		{
			previous = index;
			index = m_next[index];
		}

		if (previous != D_BT_HASH_NULL)
		{
			D_btAssert(m_next[previous] == lastPairIndex);
			m_next[previous] = m_next[lastPairIndex];
		}
		else
		{
			m_hashTable[lastHash] = m_next[lastPairIndex];
		}

		// Copy the last pair into the remove pair's spot.
		m_valueArray[pairIndex] = m_valueArray[lastPairIndex];
		m_keyArray[pairIndex] = m_keyArray[lastPairIndex];

		// Insert the last pair into the hash table
		m_next[pairIndex] = m_hashTable[lastHash];
		m_hashTable[lastHash] = pairIndex;

		m_valueArray.pop_back();
		m_keyArray.pop_back();

	}


	int size() const
	{
		return m_valueArray.size();
	}

	const D_Value* getAtIndex(int index) const
	{
		D_btAssert(index < m_valueArray.size());

		return &m_valueArray[index];
	}

	D_Value* getAtIndex(int index)
	{
		D_btAssert(index < m_valueArray.size());

		return &m_valueArray[index];
	}

	D_Value* operator[](const D_Key& key) {
		return find(key);
	}

	const D_Value*	find(const D_Key& key) const
	{
		int index = findIndex(key);
		if (index == D_BT_HASH_NULL)
		{
			return NULL;
		}
		return &m_valueArray[index];
	}

	D_Value*	find(const D_Key& key)
	{
		int index = findIndex(key);
		if (index == D_BT_HASH_NULL)
		{
			return NULL;
		}
		return &m_valueArray[index];
	}


	int	findIndex(const D_Key& key) const
	{
		unsigned int hash = key.getHash() & (m_valueArray.capacity()-1);

		if (hash >= (unsigned int)m_hashTable.size())
		{
			return D_BT_HASH_NULL;
		}

		int index = m_hashTable[hash];
		while ((index != D_BT_HASH_NULL) && key.equals(m_keyArray[index]) == false)
		{
			index = m_next[index];
		}
		return index;
	}

	void	clear()
	{
		m_hashTable.clear();
		m_next.clear();
		m_valueArray.clear();
		m_keyArray.clear();
	}

};

#endif //D_BT_HASH_MAP_H
