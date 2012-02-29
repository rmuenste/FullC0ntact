/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef _DYNAMICARRAY_H_
#define _DYNAMICARRAY_H_
#include <iostream>

namespace i3d {

/**
* @brief A dynamically allocated fixed-length array
*
* A dynamically allocated fixed-length array
*/    
template<class T>
class CDynamicArray
{

	/* private member variables */
private:
	int m_iSize;
	T   *m_Content;

public:

	/* constructors */
	CDynamicArray(void);
	CDynamicArray(int Size);
	CDynamicArray(const CDynamicArray &DynArray);

	/* public member functions */
	void Resize(int Size);
	int Size() const ;
	T&  Get(int number) const;
	T&  operator[](int number) const;
	CDynamicArray& operator=(const CDynamicArray &DynArray);
	void Destroy();
	bool Empty();

	/* destructor */
	~CDynamicArray(void);

	class iterator
	{
	public:
		
	typedef T  value_type;
	typedef T* pointer;
	typedef T& reference;
	iterator(T* curpos = NULL) : _curpos(curpos) {};

	reference operator*() {return *_curpos;}

	iterator& operator++()
	{
		_curpos=_curpos+1;
		return *this;
	}

	iterator operator++(int)
	{
		iterator old_it = *this;
		++(*this);
		return old_it;
	}

	bool operator !=(iterator rhs){return _curpos!=rhs._curpos;};

	protected:
		T* _curpos;
	};

	class const_iterator
	{
	public:
		
	typedef T  value_type;
	typedef T* pointer;
	typedef T& reference;
	const_iterator(T* curpos = NULL) : _curpos(curpos) {};

	reference operator*() const {return *_curpos;}

	const_iterator& operator++()
	{
		_curpos=_curpos+1;
		return *this;
	}

	const_iterator operator++(int)
	{
		const_iterator old_it = *this;
		++(*this);
		return old_it;
	}

	bool operator !=(const_iterator rhs){return _curpos!=rhs._curpos;};

	protected:
		T* _curpos;
	};

	iterator begin();
	iterator end();

	const_iterator begin() const;
	const_iterator end() const;
	
	friend class CDynamicArray<T>::iterator;	
	friend class CDynamicArray<T>::const_iterator;
	
};

template<class T>
void CDynamicArray<T>::Destroy()
{

	if(m_Content)
	{
		delete[] m_Content;
		m_Content = NULL;
	}

}

template<class T>
CDynamicArray<T>::CDynamicArray(void)
{

	m_Content = NULL;
	m_iSize    = 0;

}//end constructor

template<class T>
CDynamicArray<T>::~CDynamicArray(void)
{
	if(m_Content)
		delete[] m_Content;
}//end constructor

template<class T>
CDynamicArray<T>::CDynamicArray(int Size) : m_iSize(Size)
{

	m_Content = new T[Size];
	
}//end constructor

template<class T>
CDynamicArray<T>::CDynamicArray(const CDynamicArray<T> &DynArray)
{

	m_iSize = DynArray.Size();

	m_Content = new T[m_iSize];

	for(int i = 0; i < m_iSize; i++)
	{

		m_Content[i] = DynArray.Get(i);

	}//end for

}//end constructor

template<class T>
int CDynamicArray<T>::Size() const 
{

	return m_iSize;

}//end Size

template<class T>
void CDynamicArray<T>::Resize(int Size)
{

	T *newContents = new T[Size];

	for(int i = 0; i < m_iSize && i < Size;i++)
		newContents[i] = m_Content[i];

	if(m_Content)
	{
		delete[] m_Content;
		m_Content = NULL;
	}

	m_Content = newContents;
	m_iSize = Size;

}//end Resize

template<class T>
inline typename CDynamicArray<T>::iterator CDynamicArray<T>::begin()
{
	return iterator(m_Content);
}//end 

template<class T>
inline typename CDynamicArray<T>::iterator CDynamicArray<T>::end()
{
	return (m_Content + (m_iSize));
}//end

template<class T>
inline typename CDynamicArray<T>::const_iterator CDynamicArray<T>::begin() const
{
	return const_iterator(m_Content);
}//end 

template<class T>
inline typename CDynamicArray<T>::const_iterator CDynamicArray<T>::end() const
{
	return (m_Content + (m_iSize));
}//end

template<class T>
bool CDynamicArray<T>::Empty()
{

	return (m_iSize == 0);

}//end operator

template<class T>
T&  CDynamicArray<T>::Get(int number) const
{
	return m_Content[number];
}//end Get

template<class T>
T&  CDynamicArray<T>::operator[](int number) const
{
	return m_Content[number];
}//end operator

template<class T>
CDynamicArray<T>& CDynamicArray<T>::operator=(const CDynamicArray<T> &DynArray)
{

	m_iSize = DynArray.Size();

	m_Content = new T[m_iSize];

	for(int i = 0; i < m_iSize; i++)
	{

		m_Content[i] = DynArray.Get(i);

	}//end for

	return *this;

}//end operator


}

#endif
