/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef D_GEN_LIST_H
#define D_GEN_LIST_H

class D_btGEN_Link {
public:
    D_btGEN_Link() : m_next(0), m_prev(0) {}
    D_btGEN_Link(D_btGEN_Link *next, D_btGEN_Link *prev) : m_next(next), m_prev(prev) {}
    
    D_btGEN_Link *getNext() const { return m_next; }  
    D_btGEN_Link *getPrev() const { return m_prev; }  

    bool isHead() const { return m_prev == 0; }
    bool isTail() const { return m_next == 0; }

    void insertBefore(D_btGEN_Link *link) {
        m_next         = link;
        m_prev         = link->m_prev;
        m_next->m_prev = this;
        m_prev->m_next = this;
    } 

    void insertAfter(D_btGEN_Link *link) {
        m_next         = link->m_next;
        m_prev         = link;
        m_next->m_prev = this;
        m_prev->m_next = this;
    } 

    void remove() { 
        m_next->m_prev = m_prev; 
        m_prev->m_next = m_next;
    }

private:  
    D_btGEN_Link  *m_next;
    D_btGEN_Link  *m_prev;
};

class D_btGEN_List {
public:
    D_btGEN_List() : m_head(&m_tail, 0), m_tail(0, &m_head) {}

    D_btGEN_Link *getHead() const { return m_head.getNext(); } 
    D_btGEN_Link *getTail() const { return m_tail.getPrev(); } 

    void addHead(D_btGEN_Link *link) { link->insertAfter(&m_head); }
    void addTail(D_btGEN_Link *link) { link->insertBefore(&m_tail); }
    
private:
    D_btGEN_Link m_head;
    D_btGEN_Link m_tail;
};

#endif



