#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

#include "SpuFakeDma.h"
#include <LinearMath/D_btScalar.h>


///D_DoubleBuffer
template<class D_T, int size>
class D_DoubleBuffer
{
#if defined(__SPU__) || defined(USE_LIBSPE2)
	D_ATTRIBUTE_ALIGNED128( D_T m_buffer0[size] ) ;
	D_ATTRIBUTE_ALIGNED128( D_T m_buffer1[size] ) ;
#else
	D_T m_buffer0[size];
	D_T m_buffer1[size];
#endif
	
	D_T *m_frontBuffer;
	D_T *m_backBuffer;

	unsigned int m_dmaTag;
	bool m_dmaPending;
public:
	bool	isPending() const { return m_dmaPending;}
	D_DoubleBuffer();

	void init ();

	// dma get D_and put commands
	void backBufferDmaGet(D_uint64_t ea, unsigned int numBytes, unsigned int tag);
	void backBufferDmaPut(D_uint64_t ea, unsigned int numBytes, unsigned int tag);

	// gets D_pointer D_to a buffer
	D_T *getFront();
	D_T *getBack();

	// if back buffer dma was started, wait for it D_to complete
	// then move back D_to front D_and vice versa
	D_T *swapBuffers();
};

template<class D_T, int size>
D_DoubleBuffer<D_T,size>::D_DoubleBuffer()
{
	init ();
}

template<class D_T, int size>
void D_DoubleBuffer<D_T,size>::init()
{
	this->m_dmaPending = false;
	this->m_frontBuffer = &this->m_buffer0[0];
	this->m_backBuffer = &this->m_buffer1[0];
}

template<class D_T, int size>
void
D_DoubleBuffer<D_T,size>::backBufferDmaGet(D_uint64_t ea, unsigned int numBytes, unsigned int tag)
{
	m_dmaPending = true;
	m_dmaTag = tag;
	if (numBytes)
	{
		m_backBuffer = (D_T*)cellDmaLargeGetReadOnly(m_backBuffer, ea, numBytes, tag, 0, 0);
	}
}

template<class D_T, int size>
void
D_DoubleBuffer<D_T,size>::backBufferDmaPut(D_uint64_t ea, unsigned int numBytes, unsigned int tag)
{
	m_dmaPending = true;
	m_dmaTag = tag;
	D_cellDmaLargePut(m_backBuffer, ea, numBytes, tag, 0, 0);
}

template<class D_T, int size>
D_T *
D_DoubleBuffer<D_T,size>::getFront()
{
	return m_frontBuffer;
}

template<class D_T, int size>
D_T *
D_DoubleBuffer<D_T,size>::getBack()
{
	return m_backBuffer;
}

template<class D_T, int size>
D_T *
D_DoubleBuffer<D_T,size>::swapBuffers()
{
	if (m_dmaPending)
	{
		D_cellDmaWaitTagStatusAll(1<<m_dmaTag);
		m_dmaPending = false;
	}

	D_T *tmp = m_backBuffer;
	m_backBuffer = m_frontBuffer;
	m_frontBuffer = tmp;

	return m_frontBuffer;
}

#endif
