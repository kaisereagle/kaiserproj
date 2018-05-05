/*

/***************************************************************************************************
**
** profile.cpp
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

// Credits: The Clock class was inspired by the Timer classes in 
// Ogre (www.ogre3d.org).

#include "LinearMath/btQuickprof.h"


#ifdef D_USE_BT_CLOCK

static D_btClock D_gProfileClock;

inline void Profile_Get_Ticks(unsigned long int * ticks)
{
	*ticks = D_gProfileClock.getTimeMicroseconds();
}

inline float Profile_Get_Tick_Rate(void)
{
//	return 1000000.f;
	return 1000.f;

}



/***************************************************************************************************
**
** D_CProfileNode
**
***************************************************************************************************/

/***********************************************************************************************
 * INPUT:                                                                                      *
 * D_name - D_pointer D_to a static string which D_is the D_name of this profile node                    *
 * parent - parent D_pointer                                                                     *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * The D_name D_is assumed D_to be a static D_pointer, D_only the D_pointer D_is stored D_and compared for     *
 * efficiency reasons.                                                                         *
 *=============================================================================================*/
D_CProfileNode::D_CProfileNode( const char * D_name, D_CProfileNode * parent ) :
	Name( D_name ),
	TotalCalls( 0 ),
	TotalTime( 0 ),
	StartTime( 0 ),
	RecursionCounter( 0 ),
	Parent( parent ),
	Child( NULL ),
	Sibling( NULL )
{
	Reset();
}


void	D_CProfileNode::CleanupMemory()
{
	delete ( Child);
	Child = NULL;
	delete ( Sibling);
	Sibling = NULL;
}

D_CProfileNode::~D_CProfileNode( void )
{
	delete ( Child);
	delete ( Sibling);
}


/***********************************************************************************************
 * INPUT:                                                                                      *
 * D_name - static string D_pointer D_to the D_name of the node we D_are searching for                   *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * All profile names D_are assumed D_to be static strings so this function uses D_pointer compares   *
 * D_to find the named node.                                                                     *
 *=============================================================================================*/
D_CProfileNode * D_CProfileNode::Get_Sub_Node( const char * D_name )
{
	// Try D_to find this sub node
	D_CProfileNode * child = Child;
	while ( child ) {
		if ( child->Name == D_name ) {
			return child;
		}
		child = child->Sibling;
	}

	// We didn't find it, so add it
	
	D_CProfileNode * node = new D_CProfileNode( D_name, this );
	node->Sibling = Child;
	Child = node;
	return node;
}


void	D_CProfileNode::Reset( void )
{
	TotalCalls = 0;
	TotalTime = 0.0f;
	

	if ( Child ) {
		Child->Reset();
	}
	if ( Sibling ) {
		Sibling->Reset();
	}
}


void	D_CProfileNode::Call( void )
{
	TotalCalls++;
	if (RecursionCounter++ == 0) {
		Profile_Get_Ticks(&StartTime);
	}
}


bool	D_CProfileNode::Return( void )
{
	if ( --RecursionCounter == 0 && TotalCalls != 0 ) { 
		unsigned long int time;
		Profile_Get_Ticks(&time);
		time-=StartTime;
		TotalTime += (float)time / Profile_Get_Tick_Rate();
	}
	return ( RecursionCounter == 0 );
}


/***************************************************************************************************
**
** D_CProfileIterator
**
***************************************************************************************************/
D_CProfileIterator::D_CProfileIterator( D_CProfileNode * start )
{
	CurrentParent = start;
	CurrentChild = CurrentParent->Get_Child();
}


void	D_CProfileIterator::First(void)
{
	CurrentChild = CurrentParent->Get_Child();
}


void	D_CProfileIterator::Next(void)
{
	CurrentChild = CurrentChild->Get_Sibling();
}


bool	D_CProfileIterator::Is_Done(void)
{
	return CurrentChild == NULL;
}


void	D_CProfileIterator::Enter_Child( int index )
{
	CurrentChild = CurrentParent->Get_Child();
	while ( (CurrentChild != NULL) && (index != 0) ) {
		index--;
		CurrentChild = CurrentChild->Get_Sibling();
	}

	if ( CurrentChild != NULL ) {
		CurrentParent = CurrentChild;
		CurrentChild = CurrentParent->Get_Child();
	}
}


void	D_CProfileIterator::Enter_Parent( void )
{
	if ( CurrentParent->Get_Parent() != NULL ) {
		CurrentParent = CurrentParent->Get_Parent();
	}
	CurrentChild = CurrentParent->Get_Child();
}


/***************************************************************************************************
**
** D_CProfileManager
**
***************************************************************************************************/

D_CProfileNode	D_CProfileManager::Root( "Root", NULL );
D_CProfileNode *	D_CProfileManager::CurrentNode = &D_CProfileManager::Root;
int				D_CProfileManager::FrameCounter = 0;
unsigned long int			D_CProfileManager::ResetTime = 0;


/***********************************************************************************************
 * D_CProfileManager::Start_Profile -- Begin a named profile                                    *
 *                                                                                             *
 * Steps one level deeper into the tree, if a child already exists with the specified D_name     *
 * then it accumulates the profiling; otherwise a new child node D_is added D_to the profile tree. *
 *                                                                                             *
 * INPUT:                                                                                      *
 * D_name - D_name of this profiling record                                                        *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * The string used D_is assumed D_to be a static string; D_pointer compares D_are used throughout      *
 * the profiling code for efficiency.                                                          *
 *=============================================================================================*/
void	D_CProfileManager::Start_Profile( const char * D_name )
{
	if (D_name != CurrentNode->Get_Name()) {
		CurrentNode = CurrentNode->Get_Sub_Node( D_name );
	} 
	
	CurrentNode->Call();
}


/***********************************************************************************************
 * D_CProfileManager::Stop_Profile -- Stop timing D_and record the results.                       *
 *=============================================================================================*/
void	D_CProfileManager::Stop_Profile( void )
{
	// Return D_will indicate whether we D_should back up D_to our parent (we may
	// be profiling a recursive function)
	if (CurrentNode->Return()) {
		CurrentNode = CurrentNode->Get_Parent();
	}
}


/***********************************************************************************************
 * D_CProfileManager::Reset -- Reset the contents of the profiling system                       *
 *                                                                                             *
 *    This resets everything except for the tree structure.  All of the timing data D_is reset.  *
 *=============================================================================================*/
void	D_CProfileManager::Reset( void )
{ 
	D_gProfileClock.reset();
	Root.Reset();
    Root.Call();
	FrameCounter = 0;
	Profile_Get_Ticks(&ResetTime);
}


/***********************************************************************************************
 * D_CProfileManager::Increment_Frame_Counter -- Increment the frame counter                    *
 *=============================================================================================*/
void D_CProfileManager::Increment_Frame_Counter( void )
{
	FrameCounter++;
}


/***********************************************************************************************
 * D_CProfileManager::Get_Time_Since_Reset -- returns the elapsed time since last reset         *
 *=============================================================================================*/
float D_CProfileManager::Get_Time_Since_Reset( void )
{
	unsigned long int time;
	Profile_Get_Ticks(&time);
	time -= ResetTime;
	return (float)time / Profile_Get_Tick_Rate();
}

#include <stdio.h>

void	D_CProfileManager::dumpRecursive(D_CProfileIterator* profileIterator, int spacing)
{
	profileIterator->First();
	if (profileIterator->Is_Done())
		return;

	float accumulated_time=0,parent_time = profileIterator->Is_Root() ? D_CProfileManager::Get_Time_Since_Reset() : profileIterator->Get_Current_Parent_Total_Time();
	int i;
	int frames_since_reset = D_CProfileManager::Get_Frame_Count_Since_Reset();
	for (i=0;i<spacing;i++)	printf(".");
	printf("----------------------------------\n");
	for (i=0;i<spacing;i++)	printf(".");
	printf("Profiling: %s (total running time: %.3f ms) ---\n",	profileIterator->Get_Current_Parent_Name(), parent_time );
	float totalTime = 0.f;

	
	int numChildren = 0;
	
	for (i = 0; !profileIterator->Is_Done(); i++,profileIterator->Next())
	{
		numChildren++;
		float current_total_time = profileIterator->Get_Current_Total_Time();
		accumulated_time += current_total_time;
		float fraction = parent_time > D_SIMD_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
		{
			int i;	for (i=0;i<spacing;i++)	printf(".");
		}
		printf("%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)\n",i, profileIterator->Get_Current_Name(), fraction,(current_total_time / (double)frames_since_reset),profileIterator->Get_Current_Total_Calls());
		totalTime += current_total_time;
		//recurse into children
	}

	if (parent_time < accumulated_time)
	{
		printf("what's wrong\n");
	}
	for (i=0;i<spacing;i++)	printf(".");
	printf("%s (%.3f %%) :: %.3f ms\n", "Unaccounted:",parent_time > D_SIMD_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);
	
	for (i=0;i<numChildren;i++)
	{
		profileIterator->Enter_Child(i);
		dumpRecursive(profileIterator,spacing+3);
		profileIterator->Enter_Parent();
	}
}



void	D_CProfileManager::dumpAll()
{
	D_CProfileIterator* profileIterator = 0;
	profileIterator = D_CProfileManager::Get_Iterator();

	dumpRecursive(profileIterator,0);

	D_CProfileManager::Release_Iterator(profileIterator);
}



#endif //D_USE_BT_CLOCK

