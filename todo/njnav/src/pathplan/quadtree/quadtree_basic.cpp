/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_QuadTreeBasic.cpp									*/
/* Purpose: 	Define basic data struct and interface for quad-tree	*/
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// include files
#include <pathplan/quadtree/quadtree_basic.h>
#include <assert.h>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
using namespace std;

#if defined (__cplusplus)
namespace NJRobot
{

extern "C" {
#endif

//////////////////////////////////////////////////////////////////////////
// implemetation of gslist in GLib
struct SList
{
	void* data;
	SList *next;
};

void slist_free(SList* list){
	SList* p=list;
	while(p){
		SList* pnext = p->next;
		delete p;
		p = pnext;
	}
}

SList* slist_prepend(SList* list,void * data){
	SList* p = new SList;
	p->data = data;
	p->next = list;
	return p;
}

SList* slist_find(SList *list,const void *data){
	while(list){
		if(list->data==data){
			return list;
		}
		list = list->next;
	}
	return list;
}

SList* slist_remove(SList *list,const void *data){
	SList *head = new SList;
	head->next = list;

	for(SList*p = head;p->next!=NULL;p=p->next){
		if(p->next->data==data){
			SList* pnext = p->next;
			p->next = pnext->next;
			delete pnext;
			break;
		}
	}
	SList* res = head->next;
	delete head;
	return res;
}

void slist_show(SList *list){
	if(list==NULL) return;
	std::cout<<list->data;
	while(list->next){
		list = list->next;
		std::cout<<" -> "<<list->data;
	}
	std::cout<<std::endl;
}
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*					Cell related operations                             */
/************************************************************************/
/// Create a new cell with parent
stg_cell_t* stg_cell_create( stg_cell_t* parent, double x, double y, double size )
{
	stg_cell_t* cell = (stg_cell_t*)calloc( sizeof(stg_cell_t), 1);

	cell->parent = parent;
	cell->data = NULL;
	cell->x = x;
	cell->y = y;
	cell->off_x = x;
	cell->off_y = y;
	cell->size = size;

	cell->children[0] = NULL;
	cell->children[1] = NULL;
	cell->children[2] = NULL;
	cell->children[3] = NULL;

	// store bounds for fast checking
	cell->xmin = x - size/2.0;
	cell->xmax = x + size/2.0;
	cell->ymin = y - size/2.0;
	cell->ymax = y + size/2.0;

	////  [1/22/2013 cliffyin]
	//if (cell->size > 15) {
	//	stg_cell_create(cell, x - size/4.0, y - size/4.0, size/2.0);
	//	stg_cell_create(cell, x - size/4.0, y + size/4.0, size/2.0);
	//	stg_cell_create(cell, x + size/4.0, y - size/4.0, size/2.0);
	//	stg_cell_create(cell, x + size/4.0, y + size/4.0, size/2.0);
	//}

	return cell;
}

/// Frees the cell
void stg_cell_delete( stg_cell_t* cell )
{
	//////////////////////////////////////////////////////////////////////////
	// free memory used internally by GSList, this ought not
	// free the actual items pointed to by the GSList (model/object).
	if(cell->data)	slist_free((SList *)cell->data);

	//////////////////////////////////////////////////////////////////////////
	if (cell)	free( cell );

	return ;
}

/// Recursively free the quadtree of cells (rh)
void stg_cell_and_descendents_delete(stg_cell_t* cell)
{
	int i;
	if(!cell) return;

	for(i = 0; i < 4; i++) {
		if(cell->children[i]) {
			stg_cell_and_descendents_delete(cell->children[i]);
		}
	}
	stg_cell_delete(cell);

	return ;
}

/// Get the smallest cell that contains the point x,y. cell need not be the root of the tree
stg_cell_t* stg_cell_locate( stg_cell_t* cell, double x, double y )
{
	if(!cell) return NULL;

	// if x,y is NOT contained in the cell we jump to its parent
	while( !( GTE(x,cell->xmin) && LT(x,cell->xmax) && 
				GTE(y,cell->ymin) && LT(y,cell->ymax) ))
	{
		if( cell->parent )
			cell = cell->parent;
		else
			return NULL; // the point is outside the root node!
	}

	if(!cell) return NULL;

	// if we have children, we must jump down into the child
	while( cell->children[0] )
	{
		// choose the right quadrant 
		int index;
		if( LT(x,cell->x) )
			index = LT(y,cell->y) ? 0 : 2; 
		else
			index = LT(y,cell->y) ? 1 : 3; 

		cell = cell->children[index];
	}

	return cell;
}

/// Find or create the smallest cell for the given point
stg_cell_t* stg_find_or_create_leaf_cell(stg_cell_t *cell, double x1, double y1, double leaf_cell_size)
{
	// locate the leaf cell at X,Y
	cell = stg_cell_locate(cell, x1, y1 );
	if( cell == NULL )
		return NULL;

	// if the cell isn't small enough, we need to create children
	while( GT(cell->size, leaf_cell_size) )
	{
		const double halfcellsize = cell->size/2.0;
		const double delta = halfcellsize/2.0; //cell->size / 4.0
		const double x = cell->x;
		const double y = cell->y;
		const double x_minus_delta = x - delta;
		const double x_plus_delta = x + delta;
		const double y_minus_delta = y - delta;
		const double y_plus_delta = y + delta;
		int index;

		assert( cell->children[0] == NULL ); // make sure

		//delta = cell->size/4.0;

		cell->children[0] = stg_cell_create( cell,x_minus_delta,y_minus_delta,halfcellsize);
		cell->children[1] = stg_cell_create( cell,x_plus_delta,y_minus_delta,halfcellsize);
		cell->children[2] = stg_cell_create( cell,x_minus_delta,y_plus_delta,halfcellsize);
		cell->children[3] = stg_cell_create( cell,x_plus_delta,y_plus_delta,halfcellsize);

		// we have to drop down into a child. but which one?
		// which quadrant are we in?
		if( LT(x1,x) )	index = LT(y1,y) ? 0 : 2; 
		else			index = LT(y1,y) ? 1 : 3; 

		// now point to the correct child containing the point, and loop again
		cell = cell->children[ index ];           
	}

	return cell;
}

/************************************************************************/
/*					Matrix related operations                           */
/************************************************************************/
/// Create a new matrix structure
stg_matrix_t* stg_matrix_create( double ppm, double width, double height,double max_leaf_size/*=DBL_MAX*/)
{
	double sz;
	stg_matrix_t* matrix = new stg_matrix_t; //(stg_matrix_t*)calloc( sizeof(stg_matrix_t), 1 );
	assert( matrix );

	matrix->ppm = ppm; // base resolution

	// grow from the smallest cell size to find a root size that
	// encompasses the whole world
	sz = 1.0/ppm;
	while( sz < std::max(width,height) )
		sz *= 2.0;
	//sz *= 2;

	matrix->width = sz;//width;
	matrix->height = sz;//height;

	// create the root node of a quad tree
	matrix->root = stg_cell_create( NULL, sz/2.0, sz/2.0, sz );
	
	stg_matrix_adjust_leaf_size(matrix->root,max_leaf_size,width,height);

	// hash table is indexed by object pointer and a list of the cells
	// each object is rendered into
	//matrix->ptable = g_hash_table_new( g_direct_hash, g_direct_equal );
	matrix->locking_enabled = false;

	return matrix;
}

void stg_matrix_adjust_leaf_size( stg_cell_t* cell,double max_leaf_size, double width/*=-1*/, double height/*=-1 */ )
{
	if(width==-1) width = cell->size;
	if(height==-1) height = cell->size;
	if(GTE(cell->size,width) || GTE(cell->size,height) || (GT(cell->size,max_leaf_size) && cell->xmin<width && cell->ymin<height))
	{
		const double halfcellsize = cell->size/2.0;
		const double delta = halfcellsize/2.0; //cell->size / 4.0
		const double x = cell->x;
		const double y = cell->y;
		const double x_minus_delta = x - delta;
		const double x_plus_delta = x + delta;
		const double y_minus_delta = y - delta;
		const double y_plus_delta = y + delta;

		assert( cell->children[0] == NULL ); // make sure

		cell->children[0] = stg_cell_create( cell,x_minus_delta,y_minus_delta,halfcellsize);
		cell->children[1] = stg_cell_create( cell,x_plus_delta,y_minus_delta,halfcellsize);
		cell->children[2] = stg_cell_create( cell,x_minus_delta,y_plus_delta,halfcellsize);
		cell->children[3] = stg_cell_create( cell,x_plus_delta,y_plus_delta,halfcellsize);

		for(int i=0;i<4;i++){
			stg_matrix_adjust_leaf_size(cell->children[i],max_leaf_size,width,height);
		}
	}
}

/// Frees all memory allocated by the matrix; first the cells, then the cell array.
void stg_matrix_destroy( stg_matrix_t* matrix )
{
	if(matrix->root)	stg_cell_and_descendents_delete(matrix->root);
	//g_hash_table_destroy( matrix->ptable );
	//free( matrix );
	delete matrix;
}

/// Project a set of points into the matrix.
void stg_matrix_points(stg_matrix_t *matrix, double x, double y, double a,
					   stg_point_t *points, size_t npoints, void *object)
{
	size_t i;
	stg_cell_t *root;
	double res;
	int np = 0;
	int nc = 0;
	root = matrix->root;
	res = 1.0 / matrix->ppm;
	for(i = 0; i < npoints; ++i)
	{
		stg_cell_t *cell = stg_find_or_create_leaf_cell(root, x + points[i].x, y + points[i].y, res);
		if(!cell) continue;
		// if 'object' is not already in cell's data list, add it (the same
		// point could project into the same cell pretty frequently.)
		if( slist_find((SList *)cell->data, object) == NULL )
		{
			cell->data = slist_prepend((SList *)cell->data, object);
			//list = (GSList *)g_hash_table_lookup(matrix->ptable, object);
			SList* list = static_cast<SList*>(matrix->ptable.count(object)>0?matrix->ptable[object]:NULL);
			list = slist_prepend(list, cell);
			//g_hash_table_insert(matrix->ptable, object, list);
			matrix->ptable[object] = list;
			++nc;
		}
		cell->is_line = false;
		++np;
	}

	return ;
}

/// Call stg_matrix_line for each of [num_lines] lines 
void stg_matrix_lines( stg_matrix_t* matrix, stg_line_t* lines, int num_lines,void* object )
{
	size_t l;
	const double res = 1.0/matrix->ppm;

	for(l=0; l<(size_t)num_lines; ++l )
	{
		// start these values, they will be used and changed as we iterate through cells
		// for the line to pass through
		double x1 = lines[l].x1;
		double y1 = lines[l].y1;
		double x2 = lines[l].x2;
		double y2 = lines[l].y2;

		// some values used in the loop but which remain constant for this line
		const double theta = atan2( y2-y1, x2-x1 );
		const double m = tan(theta); // line gradient 

		stg_cell_t* cell = matrix->root;
		assert(cell);

		while( (GTE(fabs(x2-x1),res) || GTE(fabs(y2-y1),res)) && cell )
		{
			cell = stg_find_or_create_leaf_cell(cell, x1, y1, res);
			if(cell == NULL)
			{
				break;
			}

			// now the cell small enough, we add the object here
			cell->data = slist_prepend( (SList *)cell->data, object ); 
			cell->is_line = true;
			cell->line_angle = theta;

			// add this object the hash table
			//list = (SList *)g_hash_table_lookup( matrix->ptable, object );
			SList* list = static_cast<SList*>(matrix->ptable.count(object)>0?matrix->ptable[object]:NULL);
			list = slist_prepend( list, cell );
			//g_hash_table_insert( matrix->ptable, object, list );
			matrix->ptable[object] = list;

			if( EQ(y1,y2) ) // horizontal line
			{
				if( GT(x1,x2) ) // x1 gets smaller
					x1 = cell->xmin-0.001; // left edge
				else
					x1 = cell->xmax; // right edge		
			}
			else if( EQ(x1,x2) ) // vertical line
			{
				if( GT(y1,y2) ) // y1 gets smaller
					y1 = cell->ymin-0.001; // bottom edge
				else
					y1 = cell->ymax; // max edge		
			}
			else
			{
				double c = y1 - m * x1; // line offset

				if( GT(theta,0.0) ) // up
				{
					// ray could leave through the top edge
					// solve x for known y      
					y1 = cell->ymax; // top edge
					x1 = (y1 - c) / m;
		  
					// if the edge crossing was not in cell bounds     
					if( !(GTE(x1,cell->xmin) && LT(x1,cell->xmax)) )
					{
						// it must have left the cell on the left or right instead 
						// solve y for known x
						if( GT(theta,M_PI/2.0) ) // left side
							x1 = cell->xmin-0.00001;
						else // right side
							x1 = cell->xmax;

						y1 = m * x1 + c;
					}           
				}	 
				else // down 
				{
					// ray could leave through the bottom edge
					// solve x for known y      
					y1 = cell->ymin-0.00001; // bottom edge
					x1 = (y1 - c) / m;

					// if the edge crossing was not in cell bounds     
					if( !(GTE(x1,cell->xmin) && LT(x1,cell->xmax)) )
					{
						// it must have left the cell on the left or right instead 
						// solve y for known x	  
						if( theta < -M_PI/2.0 ) // left side
							x1 = cell->xmin-0.00001;
						else
							x1 = cell->xmax; 

						y1 = m * x1 + c;
					}
				}
			}
		}
	}

	return ;
}

/// Append to the [object] pointer to the cells on the edge of a rectangle
void stg_matrix_rectangle( stg_matrix_t* matrix, double px, double py, double pth,
						  double dx, double dy, void* object )
{
	double cx, cy, sx, sy;
	double toplx, toply, toprx, topry, botlx, botly, botrx, botry;
	stg_line_t lines[4];

	dx /= 2.0;
	dy /= 2.0;

	cx = dx * cos(pth);
	cy = dy * cos(pth);
	sx = dx * sin(pth);
	sy = dy * sin(pth);

	toplx =  px + cx - sy;
	toply =  py + sx + cy;

	toprx =  px + cx + sy;
	topry =  py + sx - cy;

	botlx =  px - cx - sy;
	botly =  py - sx + cy;

	botrx =  px - cx + sy;
	botry =  py - sx - cy;

	lines[0].x1 = toplx;
	lines[0].y1 = toply;
	lines[0].x2 = toprx;
	lines[0].y2 = topry;

	lines[1].x1 = toplx;
	lines[1].y1 = toply;
	lines[1].x2 = botlx;
	lines[1].y2 = botly;

	lines[2].x1 = toprx;
	lines[2].y1 = topry;
	lines[2].x2 = botrx;
	lines[2].y2 = botry;

	lines[3].x1 = botlx;
	lines[3].y1 = botly;
	lines[3].x2 = botrx;
	lines[3].y2 = botry;

	stg_matrix_lines( matrix, lines, 4,  object );

	return ;
}

/// Optimize the created matrix with forces by neighbouring cells
void stg_matrix_optimize( stg_matrix_t* matrix )
{
	// TODO 
	return ;
}

/// Remove the points of the object
void stg_cell_remove_object( stg_cell_t* cell, void* p )
{
	cell->data = slist_remove( (SList*)cell->data, p ); 

	// if the cell is empty and has a parent, we might be able to delete it
	if( cell->data == NULL && cell->parent )
	{
		// hop up in the tree
		cell = cell->parent;

		// while all children are empty
		while( cell && 
			!(cell->children[0]->children[0] || cell->children[0]->data ||
			cell->children[1]->children[0] || cell->children[1]->data ||
			cell->children[2]->children[0] || cell->children[2]->data ||
			cell->children[3]->children[0] || cell->children[3]->data) )
		{	      
			// detach siblings from parent and free them
			int i;
			for(i=0; i<4; i++ )
			{
				stg_cell_delete( cell->children[i] );
				cell->children[i] = NULL; 
			}

			cell = cell->parent;
		}
	}

	return ;
}

/// Remove the points of the object
void stg_matrix_remove_object( stg_matrix_t* matrix, void* object )
{
	// get the list of cells in which this object has been drawn

	//list = (GSList *)g_hash_table_lookup( matrix->ptable, object );
	SList* list = static_cast<SList*>(matrix->ptable.count(object)>0?matrix->ptable[object]:NULL);

	// remove this object from each cell in the list      
	for(SList* it = list; it; it = it->next )
	{
		//printf( "removing %p from cell %p\n", 
		//      object, it->data );
		stg_cell_remove_object( (stg_cell_t*)it->data, object );
	}

	// now free the cell list
	slist_free( list );

	// and remove the object from the hash table
	//g_hash_table_remove( matrix->ptable, object );
	matrix->ptable.erase(object);

	return ;
}

/************************************************************************/
/*					Matrix lock related                                 */
/************************************************************************/
/// Only for use from stg_world functions.
void stg_matrix_lock(stg_matrix_t *matrix)
{
	// TODO
}

/// Only for use from stg_world functions. 
void stg_matrix_unlock(stg_matrix_t *matrix)
{
	// TODO
}

/// Only for use from stg_world functions.
void stg_matrix_enable_lock(stg_matrix_t *matrix, bool enable)
{
	// TODO
}

#if defined (__cplusplus)
}

}
#endif