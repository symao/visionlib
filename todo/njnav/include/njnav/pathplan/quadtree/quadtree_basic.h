/** \file
	\brief Define basic data struct and interface for quad-tree
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#ifndef NRF_QUADTREE_BASIC_H
#define NRF_QUADTREE_BASIC_H

#include <math.h>
#include <vector>
#include <list>
#include <map>
#include <float.h>

//#include <glib/gslist.h>
namespace NJRobot
{

#if defined (__cplusplus)
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PRECISION 100000.0
/** TRUE iff A and B are equal to within PRECISION */
#define EQ(A,B) ((floor(A*PRECISION+0.5))==(floor(B*PRECISION+0.5)))
/** TRUE iff A is less than B, subject to PRECISION */
#define LT(A,B) ((floor(A*PRECISION+0.5))<(floor(B*PRECISION+0.5)))
/** TRUE iff A is greater than B, subject to PRECISION */
#define GT(A,B) ((floor(A*PRECISION+0.5))>(floor(B*PRECISION+0.5)))
/** TRUE iff A is greater than or equal B, subject to PRECISION */
#define GTE(A,B) ((floor(A*PRECISION+0.5))>=(floor(B*PRECISION+0.5)))
/** TRUE iff A is less than or equal to B, subject to PRECISION */
#define LTE(A,B) ((floor(A*PRECISION+0.5))<=(floor(B*PRECISION+0.5)))

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
typedef double stg_meters_t;
/** \brief define a point on the plane */
typedef struct
{
	stg_meters_t x, y;
} stg_point_t;

/** \brief specify a line from (x1,y1) to (x2,y2), all in meters
*/
typedef struct
{
	stg_meters_t x1, y1, x2, y2;
} stg_line_t;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/** \brief A node in the occupancy quad-tree */
typedef struct stg_cell
{
	void* data;
	double x, y;
	double off_x, off_y;	// need to be optimized with neighbouring cells
	double size;

	// bounding box
	double xmin,ymin,xmax,ymax;

	struct stg_cell* children[4];
	struct stg_cell* parent;

	bool is_line;			// if the cell is a leaf for a line (rather than a raster/bitmap cell)
	double line_angle;		// if the cell contains line data, this is the line's angle in global space. 
} stg_cell_t;

/** \brief Occupancy quadtree structure */
typedef struct stg_matrix
{
	double ppm;				// pixels per meter (1/resolution)
	double width, height;

	stg_cell_t* root;

	//GHashTable* ptable;
	std::map<void*,void*> ptable;

	bool locking_enabled;

} stg_matrix_t;

/** Model **/
struct _stg_model
{
	char* token; // automatically-generated unique ID string
	//int type; // what kind of a model am I?

	struct _stg_model *parent; // the model that owns this one, possibly NULL
	struct _stg_model *root;   // the root of the model tree this model is in

	//< Modified by symao 2016/3/3 comment
	//GPtrArray* children; // the models owned by this model  

	// a table that can contain arbitrary named data items. Can be used
	// by derived model types to store properties, and for user code
	// to associate arbitrary items with a model.
	//< Modified by symao 2016/3/3 comment
	//GHashTable* props;  

	// a datalist of stg_rtk_figs, indexed by name (string)
	//< Modified by symao 2016/3/3 comment
	//GData* figs; 

	// the number of children of each type is counted so we can
	// automatically generate names for them
	int child_type_count[256];

	int subs;     // the number of subscriptions to this model

	// allow exclusive access to this model's properties
	//< Modified by symao 2016/3/3 comment
	///pthread_mutex_t mutex;
	bool mutex_init;

	// base model type string (corresponds to 'type' value)
	char* base_type_name;

	// index used to uniquely identify this model type when there are
	// more than one of the same type (combined with base type name
	// to create unique token above)
	size_t type_instance_index;

	// name given when creating this model in the world file.
	char* instance_name;

	// this model's property gui toggle data (list of stg_property_toggle_args_t*)
	//< Modified by symao 2016/3/3 comment
	//GList* property_toggles;

	// Matrix cell containing this model's center, or NULL if unknown. This is
	// a cache to improve performance when doing raycasting from near this
	// location.
	struct stg_cell *current_matrix_cell;
};
typedef _stg_model stg_model_t;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*					Cell related operations                             */
/************************************************************************/
/// Create a new cell with parent
stg_cell_t* stg_cell_create( stg_cell_t* parent, double x, double y, double size );

/// Frees the cell
void stg_cell_delete( stg_cell_t* cell );

/// Recursively free the quadtree of cells (rh)
void stg_cell_and_descendents_delete(stg_cell_t* cell);

/// Get the smallest cell that contains the point x,y. cell need not be the root of the tree
stg_cell_t* stg_cell_locate( stg_cell_t* cell, double x, double y );

/// Find or create the smallest cell for the given point
stg_cell_t* stg_find_or_create_leaf_cell(stg_cell_t *startcell, double x, double y, double leaf_cell_size);

/************************************************************************/
/*					Matrix related operations                           */
/************************************************************************/
/// Create a new matrix structure
stg_matrix_t* stg_matrix_create( double ppm, double width, double height,double max_leaf_size=DBL_MAX);

/// Adjust the max leaf size to max_lefe_size
void stg_matrix_adjust_leaf_size( stg_cell_t* cell,double max_leaf_size, double width=-1, double height=-1  );

/// Frees all memory allocated by the matrix; first the cells, then the cell array.
void stg_matrix_destroy( stg_matrix_t* matrix );

/// Project a set of points into the matrix.
void stg_matrix_points(stg_matrix_t *matrix, double x, double y, double a,
					   stg_point_t *points, size_t npoints, void *object);

/// Call stg_matrix_line for each of [num_lines] lines 
void stg_matrix_lines( stg_matrix_t* matrix, stg_line_t* lines, int num_lines,void* object );

/// Append to the [object] pointer to the cells on the edge of a rectangle
void stg_matrix_rectangle( stg_matrix_t* matrix, double px, double py, double pth,
						  double dx, double dy, void* object );

/// Optimize the created matrix with forces by neighbouring cells
void stg_matrix_optimize( stg_matrix_t* matrix );

/// Remove the points of the object
void stg_cell_remove_object( stg_cell_t* cell, void* p );

/// Remove the points of the object in matrix
void stg_matrix_remove_object( stg_matrix_t* matrix, void* object );

/************************************************************************/
/*					Matrix lock related                                 */
/************************************************************************/
/// Only for use from stg_world functions.
void stg_matrix_lock(stg_matrix_t *matrix);

/// Only for use from stg_world functions. 
void stg_matrix_unlock(stg_matrix_t *matrix);

/// Only for use from stg_world functions.
void stg_matrix_enable_lock(stg_matrix_t *matrix, bool enable);

#if defined (__cplusplus)
}
#endif


}

#endif	// ~NRF_QUADTREE_BASIC_H