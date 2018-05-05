#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <map>

// スマートポインタ
template <typename TYPE>
struct SPtr
{
	typedef boost::shared_ptr<TYPE> Type;
};

template <typename TYPE, size_t SIZE>
struct Array
{
	typedef boost::array<TYPE, SIZE> Type;
};


//------------------------------------------
class BinObject : boost::noncopyable
{
public:
	BinObject() : sizeBuff(0), ptrBuff(0){}
	BinObject(size_t s, size_t align=8) : sizeBuff(0), ptrBuff(0){
		Alloc(s,align);
	}
	~BinObject(){
		Free();
	}

	bool Alloc(size_t s, size_t align=8)
	{
		Free();
		ptrBuff = _aligned_malloc(s, align);
		sizeBuff = s;
		if(!ptrBuff)sizeBuff = 0;
		return ptrBuff!=nullptr;
	}
	void Free()
	{
		if(ptrBuff){
			_aligned_free(ptrBuff);
		}
		ptrBuff = nullptr;
	}

	void* get() { return ptrBuff;}
	size_t size() const { return sizeBuff;}
private:
	size_t sizeBuff;
	void* ptrBuff;
};

//----------------------------------------------
// リソースライブラリ
template <typename RES_TYPE>
class ResLibrary
{
public:
	// ライブラリに追加
	bool AddRes(const std::string& name, const RES_TYPE& res)
	{
		return mapRes.insert(std::pair<std::string,RES_TYPE>(name, res)).second;
	}

	// リソース取得
	bool getRes(const std::string& name, RES_TYPE& res) const
	{
		std::map<std::string, RES_TYPE>::const_iterator it = mapRes.find(name);
		if(it == mapRes.end())return false;
		res = it->second;
		return true;
	}
private:
	std::map<std::string, RES_TYPE> mapRes;
};
