/*

ply2lrt converter--a trivial modification of:

ply2iv, Convert a PLY file to an Inventor file.

Greg Turk

-----------------------------------------------------------------------

Copyright (c) 1998 Georgia Institute of Technology.  All rights reserved.   
  
Permission to use, copy, modify and distribute this software and its   
documentation for any purpose is hereby granted without fee, provided   
that the above copyright notice and this permission notice appear in   
all copies of this software and that you do not sell the software.   
  
THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,   
EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY   
WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.   

*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ply.h>


/* vertex and face definitions for a polygonal object */

typedef struct Vertex {
  float x,y,z;
  float r,g,b;
  float nx,ny,nz;
  void *other_props;       /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
  void *other_props;       /* other properties */
} Face;

char *elem_names[] = { /* list of the elements in the object */
  "vertex", "face"
};

PlyProperty vert_props[] = { /* list of property information for a vertex */
  {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
  {"r", Float32, Float32, offsetof(Vertex,r), 0, 0, 0, 0},
  {"g", Float32, Float32, offsetof(Vertex,g), 0, 0, 0, 0},
  {"b", Float32, Float32, offsetof(Vertex,b), 0, 0, 0, 0},
  {"nx", Float32, Float32, offsetof(Vertex,nx), 0, 0, 0, 0},
  {"ny", Float32, Float32, offsetof(Vertex,ny), 0, 0, 0, 0},
  {"nz", Float32, Float32, offsetof(Vertex,nz), 0, 0, 0, 0},
};

PlyProperty face_props[] = { /* list of property information for a face */
  {"vertex_indices", Int32, Int32, offsetof(Face,verts),
   1, Uint8, Uint8, offsetof(Face,nverts)},
};


/*** the PLY object ***/

static int nverts,nfaces;
static Vertex **vlist;
static Face **flist;

static PlyOtherProp *vert_other,*face_other;

static int per_vertex_color = 0;
static int has_normals = 0;

void usage(char *progname);
void read_file(void);
void write_lrt(void);


/******************************************************************************
Main program.
******************************************************************************/

int
main(int argc, char *argv[])
{
  char *s;
  char *progname;

  progname = argv[0];

  while (--argc > 0 && (*++argv)[0]=='-') {
    for (s = argv[0]+1; *s; s++)
      switch (*s) {
        default:
          usage (progname);
          exit (-1);
          break;
      }
  }

  read_file();
  write_lrt();
  return 0;
}


/******************************************************************************
Print out usage information.
******************************************************************************/

void
usage(char *progname)
{
  fprintf (stderr, "usage: %s [flags] <in.ply >out.iv\n", progname);
}


/******************************************************************************
Read in the PLY file from standard in.
******************************************************************************/

void
read_file(void)
{
  int i,j;
  int elem_count;
  char *elem_name;
  PlyFile *in_ply;

  /*** Read in the original PLY object ***/

  in_ply  = read_ply (stdin);

  for (i = 0; i < in_ply->num_elem_types; i++) {

    /* prepare to read the i'th list of elements */
    elem_name = setup_element_read_ply (in_ply, i, &elem_count);

    if (equal_strings ("vertex", elem_name)) {

      /* create a vertex list to hold all the vertices */
      vlist = (Vertex **) malloc (sizeof (Vertex *) * elem_count);
      nverts = elem_count;

      /* set up for getting vertex elements */

      setup_property_ply (in_ply, &vert_props[0]);
      setup_property_ply (in_ply, &vert_props[1]);
      setup_property_ply (in_ply, &vert_props[2]);

      for (j = 0; j < in_ply->elems[i]->nprops; j++) {
	PlyProperty *prop;
	prop = in_ply->elems[i]->props[j];
	if (equal_strings ("r", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[3]);
	  per_vertex_color = 1;
	}
	if (equal_strings ("g", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[4]);
	  per_vertex_color = 1;
	}
	if (equal_strings ("b", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[5]);
	  per_vertex_color = 1;
	}
	if (equal_strings ("nx", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[6]);
	  has_normals = 1;
	}
	if (equal_strings ("ny", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[7]);
	  has_normals = 1;
	}
	if (equal_strings ("nz", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[8]);
	  has_normals = 1;
	}
      }

      vert_other = get_other_properties_ply (in_ply, 
					     offsetof(Vertex,other_props));

      /* grab all the vertex elements */
      for (j = 0; j < elem_count; j++) {
        vlist[j] = (Vertex *) malloc (sizeof (Vertex));
	vlist[j]->r = 1;
	vlist[j]->g = 1;
	vlist[j]->b = 1;
        get_element_ply (in_ply, (void *) vlist[j]);
      }
    }
    else if (equal_strings ("face", elem_name)) {

      /* create a list to hold all the face elements */
      flist = (Face **) malloc (sizeof (Face *) * elem_count);
      nfaces = elem_count;

      /* set up for getting face elements */

      setup_property_ply (in_ply, &face_props[0]);
      face_other = get_other_properties_ply (in_ply, 
					     offsetof(Face,other_props));

      /* grab all the face elements */
      for (j = 0; j < elem_count; j++) {
        flist[j] = (Face *) malloc (sizeof (Face));
        get_element_ply (in_ply, (void *) flist[j]);
      }
    }
    else
      get_other_element_ply (in_ply);
  }

  close_ply (in_ply);
  free_ply (in_ply);
}


/******************************************************************************
Write out a lrt file.
******************************************************************************/

void
write_lrt(void)
{
  int i,j;

  printf ("Shape \"trianglemesh\" \n");
  printf ("  \"point P\" [\n");
  for (i = 0; i < nverts; i++)
    printf ("    %g %g %g\n", vlist[i]->x, vlist[i]->y, vlist[i]->z);
  printf ("  ]\n");

  /* if we have them, write surface normals */

  if (has_normals) {
    printf ("\"normal N\" [\n");
    for (i = 0; i < nverts; i++)
      printf ("    %g %g %g\n", vlist[i]->nx, vlist[i]->ny, vlist[i]->nz);
    printf ("  ]\n");
    printf ("\n");
  }

  printf ("\"integer indices\" [\n");
  for (i = 0; i < nfaces; i++) {
      /* triangulate the faces... */
      int nv = flist[i]->nverts;
      for (j = 0; j < nv-2; ++j)
	  printf (" %d %d %d\n", flist[i]->verts[0], flist[i]->verts[j+1], flist[i]->verts[j+2]);
  }
  printf ("  ]\n");

  printf ("\n");
}

