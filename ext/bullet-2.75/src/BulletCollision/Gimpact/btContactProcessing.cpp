
/*
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "btContactProcessing.h"

#define D_MAX_COINCIDENT 8

struct D_CONTACT_KEY_TOKEN
{
	unsigned int m_key;
	int m_value;
	D_CONTACT_KEY_TOKEN()
    {
    }

    D_CONTACT_KEY_TOKEN(unsigned int key,int token)
    {
    	m_key = key;
    	m_value =  token;
    }

    D_CONTACT_KEY_TOKEN(const D_CONTACT_KEY_TOKEN& rtoken)
    {
    	m_key = rtoken.m_key;
    	m_value = rtoken.m_value;
    }

    inline bool operator <(const D_CONTACT_KEY_TOKEN& other) const
	{
		return (m_key < other.m_key);
	}

	inline bool operator >(const D_CONTACT_KEY_TOKEN& other) const
	{
		return (m_key > other.m_key);
	}

};

class D_CONTACT_KEY_TOKEN_COMP
{
	public:

		bool operator() ( const D_CONTACT_KEY_TOKEN& a, const D_CONTACT_KEY_TOKEN& b )
		{
			return ( a < b );
		}
};


void D_btContactArray::merge_contacts(
	const D_btContactArray & contacts, bool normal_contact_average)
{
	clear();

	int i;
	if(contacts.size()==0) return;


	if(contacts.size()==1)
	{
		push_back(contacts[0]);
		return;
	}

	D_btAlignedObjectArray<D_CONTACT_KEY_TOKEN> keycontacts;

	keycontacts.reserve(contacts.size());

	//fill key contacts

	for ( i = 0;i<contacts.size() ;i++ )
	{
		keycontacts.push_back(D_CONTACT_KEY_TOKEN(contacts[i].calc_key_contact(),i));
	}

	//sort keys
	keycontacts.quickSort(D_CONTACT_KEY_TOKEN_COMP());

	// Merge contacts
	int coincident_count=0;
	D_btVector3 coincident_normals[D_MAX_COINCIDENT];

	unsigned int last_key = keycontacts[0].m_key;
	unsigned int key = 0;

	push_back(contacts[keycontacts[0].m_value]);

	D_GIM_CONTACT * pcontact = &(*this)[0];

	for( i=1;i<keycontacts.size();i++)
	{
	    key = keycontacts[i].m_key;
		const D_GIM_CONTACT * scontact = &contacts[keycontacts[i].m_value];

		if(last_key ==  key)//same points
		{
			//merge contact
			if(pcontact->m_depth - D_CONTACT_DIFF_EPSILON > scontact->m_depth)//)
			{
				*pcontact = *scontact;
                coincident_count = 0;
			}
			else if(normal_contact_average)
			{
				if(D_btFabs(pcontact->m_depth - scontact->m_depth)<D_CONTACT_DIFF_EPSILON)
                {
                    if(coincident_count<D_MAX_COINCIDENT)
                    {
                    	coincident_normals[coincident_count] = scontact->m_normal;
                        coincident_count++;
                    }
                }
			}
		}
		else
		{//add new contact

		    if(normal_contact_average && coincident_count>0)
		    {
		    	pcontact->interpolate_normals(coincident_normals,coincident_count);
		        coincident_count = 0;
		    }

		    push_back(*scontact);
		    pcontact = &(*this)[this->size()-1];
        }
		last_key = key;
	}
}

void D_btContactArray::merge_contacts_unique(const D_btContactArray & contacts)
{
	clear();

	if(contacts.size()==0) return;

	if(contacts.size()==1)
	{
		push_back(contacts[0]);
		return;
	}

	D_GIM_CONTACT average_contact = contacts[0];

	for (int i=1;i<contacts.size() ;i++ )
	{
		average_contact.m_point += contacts[i].m_point;
		average_contact.m_normal += contacts[i].m_normal * contacts[i].m_depth;
	}

	//divide
	D_btScalar divide_average = 1.0f/((D_btScalar)contacts.size());

	average_contact.m_point *= divide_average;

	average_contact.m_normal *= divide_average;

	average_contact.m_depth = average_contact.m_normal.length();

	average_contact.m_normal /= average_contact.m_depth;

}

