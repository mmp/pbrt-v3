/*

The interface routines for reading and writing PLY polygon files.

Greg Turk

---------------------------------------------------------------

A PLY file contains a single polygonal _object_.

An object is composed of lists of _elements_.  Typical elements are
vertices, faces, edges and materials.

Each type of element for a given object has one or more _properties_
associated with the element type.  For instance, a vertex element may
have as properties the floating-point values x,y,z and the three unsigned
chars representing red, green and blue.

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
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ply.h>

char *type_names[] = {  /* names of scalar types */
"invalid",
"int8", "int16", "int32", "uint8", "uint16", "uint32", "float32", "float64",
};

char *old_type_names[] = {  /* old names of types for backward compatability */
"invalid",
"char", "short", "int", "uchar", "ushort", "uint", "float", "double",
};

int ply_type_size[] = {
  0, 1, 2, 4, 1, 2, 4, 4, 8
};

#define NO_OTHER_PROPS  -1

#define DONT_STORE_PROP  0
#define STORE_PROP       1

#define OTHER_PROP       0
#define NAMED_PROP       1

/* returns 1 if strings are equal, 0 if not */
int equal_strings(char *, char *);

/* find an element in a plyfile's list */
PlyElement *find_element(PlyFile *, char *);

/* find a property in an element's list */
PlyProperty *find_property(PlyElement *, char *, int *);

/* write to a file the word describing a PLY file data type */
void write_scalar_type (FILE *, int);

/* read a line from a file and break it up into separate words */
char **get_words(FILE *, int *, char **);

/* write an item to a file */
void write_binary_item(FILE *, int, unsigned int, double, int);
void write_ascii_item(FILE *, int, unsigned int, double, int);

/* add information to a PLY file descriptor */
void add_element(PlyFile *, char **, int);
void add_property(PlyFile *, char **, int);
void add_comment(PlyFile *, char *);
void add_obj_info(PlyFile *, char *);

/* copy a property */
void copy_property(PlyProperty *, PlyProperty *);

/* store a value into where a pointer and a type specify */
void store_item(char *, int, int, unsigned int, double);

/* return the value of a stored item */
void get_stored_item( void *, int, int *, unsigned int *, double *);

/* return the value stored in an item, given ptr to it and its type */
double get_item_value(char *, int);

/* get binary or ascii item and store it according to ptr and type */
void get_ascii_item(char *, int, int *, unsigned int *, double *);
void get_binary_item(FILE *, int, int *, unsigned int *, double *);

/* get a bunch of elements from a file */
void ascii_get_element(PlyFile *, char *);
void binary_get_element(PlyFile *, char *);

/* memory allocation */
static char *my_alloc(int, int, char *);


/*************/
/*  Writing  */
/*************/


/******************************************************************************
Given a file pointer, get ready to write PLY data to the file.

Entry:
  fp         - the given file pointer
  nelems     - number of elements in object
  elem_names - list of element names
  file_type  - file type, either ascii or binary

Exit:
  returns a pointer to a PlyFile, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *ply_write(
  FILE *fp,
  int nelems,
  char **elem_names,
  int file_type
)
{
  int i;
  PlyFile *plyfile;
  PlyElement *elem;

  /* check for NULL file pointer */
  if (fp == NULL)
    return (NULL);

  /* create a record for this object */

  plyfile = (PlyFile *) myalloc (sizeof (PlyFile));
  plyfile->file_type = file_type;
  plyfile->num_comments = 0;
  plyfile->num_obj_info = 0;
  plyfile->num_elem_types = nelems;
  plyfile->version = 1.0;
  plyfile->fp = fp;
  plyfile->other_elems = NULL;

  /* tuck aside the names of the elements */

  plyfile->elems = (PlyElement **) myalloc (sizeof (PlyElement *) * nelems);
  for (i = 0; i < nelems; i++) {
    elem = (PlyElement *) myalloc (sizeof (PlyElement));
    plyfile->elems[i] = elem;
    elem->name = strdup (elem_names[i]);
    elem->num = 0;
    elem->nprops = 0;
  }

  /* return pointer to the file descriptor */
  return (plyfile);
}


/******************************************************************************
Open a polygon file for writing.

Entry:
  filename   - name of file to read from
  nelems     - number of elements in object
  elem_names - list of element names
  file_type  - file type, either ascii or binary

Exit:
  returns a file identifier, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *open_for_writing_ply(
  char *filename,
  int nelems,
  char **elem_names,
  int file_type
)
{
  PlyFile *plyfile;
  char *name;
  FILE *fp;

  /* tack on the extension .ply, if necessary */

  name = (char *) myalloc (sizeof (char) * (strlen (filename) + 5));
  strcpy (name, filename);
  if (strlen (name) < 4 ||
      strcmp (name + strlen (name) - 4, ".ply") != 0)
      strcat (name, ".ply");

  /* open the file for writing */

  fp = fopen (name, "w");
  if (fp == NULL) {
    return (NULL);
  }

  /* create the actual PlyFile structure */

  plyfile = ply_write (fp, nelems, elem_names, file_type);
  if (plyfile == NULL)
    return (NULL);

  /* return pointer to the file descriptor */
  return (plyfile);
}


/******************************************************************************
Describe an element, including its properties and how many will be written
to the file.

Entry:
  plyfile   - file identifier
  elem_name - name of element that information is being specified about
  nelems    - number of elements of this type to be written
  nprops    - number of properties contained in the element
  prop_list - list of properties
******************************************************************************/

void element_layout_ply(
  PlyFile *plyfile,
  char *elem_name,
  int nelems,
  int nprops,
  PlyProperty *prop_list
)
{
  int i;
  PlyElement *elem;
  PlyProperty *prop;

  /* look for appropriate element */
  elem = find_element (plyfile, elem_name);
  if (elem == NULL) {
    fprintf(stderr,"element_layout_ply: can't find element '%s'\n",elem_name);
    exit (-1);
  }

  elem->num = nelems;

  /* copy the list of properties */

  elem->nprops = nprops;
  elem->props = (PlyProperty **) myalloc (sizeof (PlyProperty *) * nprops);
  elem->store_prop = (char *) myalloc (sizeof (char) * nprops);

  for (i = 0; i < nprops; i++) {
    prop = (PlyProperty *) myalloc (sizeof (PlyProperty));
    elem->props[i] = prop;
    elem->store_prop[i] = NAMED_PROP;
    copy_property (prop, &prop_list[i]);
  }
}


/******************************************************************************
Describe a property of an element.

Entry:
  plyfile   - file identifier
  elem_name - name of element that information is being specified about
  prop      - the new property
******************************************************************************/

void ply_describe_property(
  PlyFile *plyfile,
  char *elem_name,
  PlyProperty *prop
)
{
  PlyElement *elem;
  PlyProperty *elem_prop;

  /* look for appropriate element */
  elem = find_element (plyfile, elem_name);
  if (elem == NULL) {
    fprintf(stderr, "ply_describe_property: can't find element '%s'\n",
            elem_name);
    return;
  }

  /* create room for new property */

  if (elem->nprops == 0) {
    elem->props = (PlyProperty **) myalloc (sizeof (PlyProperty *));
    elem->store_prop = (char *) myalloc (sizeof (char));
    elem->nprops = 1;
  }
  else {
    elem->nprops++;
    elem->props = (PlyProperty **)
                  realloc (elem->props, sizeof (PlyProperty *) * elem->nprops);
    elem->store_prop = (char *)
                  realloc (elem->store_prop, sizeof (char) * elem->nprops);
  }

  /* copy the new property */

  elem_prop = (PlyProperty *) myalloc (sizeof (PlyProperty));
  elem->props[elem->nprops - 1] = elem_prop;
  elem->store_prop[elem->nprops - 1] = NAMED_PROP;
  copy_property (elem_prop, prop);
}


/******************************************************************************
State how many of a given element will be written.

Entry:
  plyfile   - file identifier
  elem_name - name of element that information is being specified about
  nelems    - number of elements of this type to be written
******************************************************************************/

void element_count_ply(
  PlyFile *plyfile,
  char *elem_name,
  int nelems
)
{
  PlyElement *elem;

  /* look for appropriate element */
  elem = find_element (plyfile, elem_name);
  if (elem == NULL) {
    fprintf(stderr,"element_count_ply: can't find element '%s'\n",elem_name);
    exit (-1);
  }

  elem->num = nelems;
}


/******************************************************************************
Signal that we've described everything a PLY file's header and that the
header should be written to the file.

Entry:
  plyfile - file identifier
******************************************************************************/

void header_complete_ply(PlyFile *plyfile)
{
  int i,j;
  FILE *fp = plyfile->fp;
  PlyElement *elem;
  PlyProperty *prop;

  fprintf (fp, "ply\n");

  switch (plyfile->file_type) {
    case PLY_ASCII:
      fprintf (fp, "format ascii 1.0\n");
      break;
    case PLY_BINARY_BE:
      fprintf (fp, "format binary_big_endian 1.0\n");
      break;
    case PLY_BINARY_LE:
      fprintf (fp, "format binary_little_endian 1.0\n");
      break;
    default:
      fprintf (stderr, "ply_header_complete: bad file type = %d\n",
               plyfile->file_type);
      exit (-1);
  }

  /* write out the comments */

  for (i = 0; i < plyfile->num_comments; i++)
    fprintf (fp, "comment %s\n", plyfile->comments[i]);

  /* write out object information */

  for (i = 0; i < plyfile->num_obj_info; i++)
    fprintf (fp, "obj_info %s\n", plyfile->obj_info[i]);

  /* write out information about each element */

  for (i = 0; i < plyfile->num_elem_types; i++) {

    elem = plyfile->elems[i];
    fprintf (fp, "element %s %d\n", elem->name, elem->num);

    /* write out each property */
    for (j = 0; j < elem->nprops; j++) {
      prop = elem->props[j];
      if (prop->is_list == PLY_LIST) {
        fprintf (fp, "property list ");
        write_scalar_type (fp, prop->count_external);
        fprintf (fp, " ");
        write_scalar_type (fp, prop->external_type);
        fprintf (fp, " %s\n", prop->name);
      }
      else if (prop->is_list == PLY_STRING) {
        fprintf (fp, "property string");
        fprintf (fp, " %s\n", prop->name);
      }
      else {
        fprintf (fp, "property ");
        write_scalar_type (fp, prop->external_type);
        fprintf (fp, " %s\n", prop->name);
      }
    }
  }

  fprintf (fp, "end_header\n");
}


/******************************************************************************
Specify which elements are going to be written.  This should be called
before a call to the routine ply_put_element().

Entry:
  plyfile   - file identifier
  elem_name - name of element we're talking about
******************************************************************************/

void put_element_setup_ply(PlyFile *plyfile, char *elem_name)
{
  PlyElement *elem;

  elem = find_element (plyfile, elem_name);
  if (elem == NULL) {
    fprintf(stderr, "put_element_setup_ply: can't find element '%s'\n", elem_name);
    exit (-1);
  }

  plyfile->which_elem = elem;
}


/******************************************************************************
Write an element to the file.  This routine assumes that we're
writing the type of element specified in the last call to the routine
put_element_setup_ply().

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to the element
******************************************************************************/

void put_element_ply(PlyFile *plyfile, void *elem_ptr)
{
  int j,k;
  FILE *fp = plyfile->fp;
  PlyElement *elem;
  PlyProperty *prop;
  char *item;
  char *elem_data;
  char **item_ptr;
  int list_count;
  int item_size;
  int int_val;
  unsigned int uint_val;
  double double_val;
  char **other_ptr;

  elem = plyfile->which_elem;
  elem_data = (char *) elem_ptr;
  other_ptr = (char **) (((char *) elem_ptr) + elem->other_offset);

  /* write out either to an ascii or binary file */

  if (plyfile->file_type == PLY_ASCII) {

    /* write an ascii file */

    /* write out each property of the element */
    for (j = 0; j < elem->nprops; j++) {

      prop = elem->props[j];

      if (elem->store_prop[j] == OTHER_PROP)
        elem_data = *other_ptr;
      else
        elem_data = (char *) elem_ptr;

      if (prop->is_list == PLY_LIST) {  /* list */
        item = elem_data + prop->count_offset;
        get_stored_item ((void *) item, prop->count_internal,
                         &int_val, &uint_val, &double_val);
        write_ascii_item (fp, int_val, uint_val, double_val,
                          prop->count_external);
        list_count = uint_val;
        item_ptr = (char **) (elem_data + prop->offset);
        item = item_ptr[0];
        item_size = ply_type_size[prop->internal_type];
        for (k = 0; k < list_count; k++) {
          get_stored_item ((void *) item, prop->internal_type,
                           &int_val, &uint_val, &double_val);
          write_ascii_item (fp, int_val, uint_val, double_val,
                            prop->external_type);
          item += item_size;
        }
      }
      else if (prop->is_list == PLY_STRING) {  /* string */
	char **str;
        item = elem_data + prop->offset;
	str = (char **) item;
	fprintf (fp, "\"%s\"", *str);
      }
      else {                                  /* scalar */
        item = elem_data + prop->offset;
        get_stored_item ((void *) item, prop->internal_type,
                         &int_val, &uint_val, &double_val);
        write_ascii_item (fp, int_val, uint_val, double_val,
                          prop->external_type);
      }
    }

    fprintf (fp, "\n");
  }
  else {

    /* write a binary file */

    /* write out each property of the element */
    for (j = 0; j < elem->nprops; j++) {
      prop = elem->props[j];
      if (elem->store_prop[j] == OTHER_PROP)
        elem_data = *other_ptr;
      else
        elem_data = (char *) elem_ptr;
      if (prop->is_list == PLY_LIST) {   /* list */
        item = elem_data + prop->count_offset;
        item_size = ply_type_size[prop->count_internal];
        get_stored_item ((void *) item, prop->count_internal,
                         &int_val, &uint_val, &double_val);
        write_binary_item (fp, int_val, uint_val, double_val,
                           prop->count_external);
        list_count = uint_val;
        item_ptr = (char **) (elem_data + prop->offset);
        item = item_ptr[0];
        item_size = ply_type_size[prop->internal_type];
        for (k = 0; k < list_count; k++) {
          get_stored_item ((void *) item, prop->internal_type,
                           &int_val, &uint_val, &double_val);
          write_binary_item (fp, int_val, uint_val, double_val,
                             prop->external_type);
          item += item_size;
        }
      }
      else if (prop->is_list == PLY_STRING) {   /* string */
	int len;
	char **str;
        item = elem_data + prop->offset;
	str = (char **) item;

	/* write the length */
	len = strlen(*str) + 1;
	fwrite (&len, sizeof(int), 1, fp);

	/* write the string, including the null character */
	fwrite (*str, len, 1, fp);
      }
      else {                   /* scalar */
        item = elem_data + prop->offset;
        item_size = ply_type_size[prop->internal_type];
        get_stored_item ((void *) item, prop->internal_type,
                         &int_val, &uint_val, &double_val);
        write_binary_item (fp, int_val, uint_val, double_val,
                           prop->external_type);
      }
    }

  }
}






/*************/
/*  Reading  */
/*************/



/******************************************************************************
Given a file pointer, get ready to read PLY data from the file.

Entry:
  fp - the given file pointer

Exit:
  nelems     - number of elements in object
  elem_names - list of element names
  returns a pointer to a PlyFile, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *ply_read(FILE *fp, int *nelems, char ***elem_names)
{
  int i,j;
  PlyFile *plyfile;
  int nwords;
  char **words;
  int found_format = 0;
  char **elist;
  PlyElement *elem;
  char *orig_line;

  /* check for NULL file pointer */
  if (fp == NULL)
    return (NULL);

  /* create record for this object */

  plyfile = (PlyFile *) myalloc (sizeof (PlyFile));
  plyfile->num_elem_types = 0;
  plyfile->comments = NULL;
  plyfile->num_comments = 0;
  plyfile->obj_info = NULL;
  plyfile->num_obj_info = 0;
  plyfile->fp = fp;
  plyfile->other_elems = NULL;
  plyfile->rule_list = NULL;

  /* read and parse the file's header */

  words = get_words (plyfile->fp, &nwords, &orig_line);
  if (!words || !equal_strings (words[0], "ply"))
    return (NULL);

  while (words) {

    /* parse words */

    if (equal_strings (words[0], "format")) {
      if (nwords != 3)
        return (NULL);
      if (equal_strings (words[1], "ascii"))
        plyfile->file_type = PLY_ASCII;
      else if (equal_strings (words[1], "binary_big_endian"))
        plyfile->file_type = PLY_BINARY_BE;
      else if (equal_strings (words[1], "binary_little_endian"))
        plyfile->file_type = PLY_BINARY_LE;
      else
        return (NULL);
      plyfile->version = atof (words[2]);
      found_format = 1;
    }
    else if (equal_strings (words[0], "element"))
      add_element (plyfile, words, nwords);
    else if (equal_strings (words[0], "property"))
      add_property (plyfile, words, nwords);
    else if (equal_strings (words[0], "comment"))
      add_comment (plyfile, orig_line);
    else if (equal_strings (words[0], "obj_info"))
      add_obj_info (plyfile, orig_line);
    else if (equal_strings (words[0], "end_header"))
      break;

    /* free up words space */
    free (words);

    words = get_words (plyfile->fp, &nwords, &orig_line);
  }

  /* create tags for each property of each element, to be used */
  /* later to say whether or not to store each property for the user */

  for (i = 0; i < plyfile->num_elem_types; i++) {
    elem = plyfile->elems[i];
    elem->store_prop = (char *) myalloc (sizeof (char) * elem->nprops);
    for (j = 0; j < elem->nprops; j++)
      elem->store_prop[j] = DONT_STORE_PROP;
    elem->other_offset = NO_OTHER_PROPS; /* no "other" props by default */
  }

  /* set return values about the elements */

  elist = (char **) myalloc (sizeof (char *) * plyfile->num_elem_types);
  for (i = 0; i < plyfile->num_elem_types; i++)
    elist[i] = strdup (plyfile->elems[i]->name);

  *elem_names = elist;
  *nelems = plyfile->num_elem_types;

  /* return a pointer to the file's information */

  return (plyfile);
}


/******************************************************************************
Open a polygon file for reading.

Entry:
  filename - name of file to read from

Exit:
  nelems     - number of elements in object
  elem_names - list of element names
  file_type  - file type, either ascii or binary
  version    - version number of PLY file
  returns a file identifier, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *ply_open_for_reading(
  char *filename,
  int *nelems,
  char ***elem_names,
  int *file_type,
  float *version
)
{
  FILE *fp;
  PlyFile *plyfile;
  char *name;

  /* tack on the extension .ply, if necessary */

  name = (char *) myalloc (sizeof (char) * (strlen (filename) + 5));
  strcpy (name, filename);
  if (strlen (name) < 4 ||
      strcmp (name + strlen (name) - 4, ".ply") != 0)
      strcat (name, ".ply");

  /* open the file for reading */

  fp = fopen (name, "r");
  if (fp == NULL)
    return (NULL);

  /* create the PlyFile data structure */

  plyfile = ply_read (fp, nelems, elem_names);

  /* determine the file type and version */

  *file_type = plyfile->file_type;
  *version = plyfile->version;

  /* return a pointer to the file's information */

  return (plyfile);
}


/******************************************************************************
Get information about a particular element.

Entry:
  plyfile   - file identifier
  elem_name - name of element to get information about

Exit:
  nelems   - number of elements of this type in the file
  nprops   - number of properties
  returns a list of properties, or NULL if the file doesn't contain that elem
******************************************************************************/

PlyProperty **get_element_description_ply(
  PlyFile *plyfile,
  char *elem_name,
  int *nelems,
  int *nprops
)
{
  int i;
  PlyElement *elem;
  PlyProperty *prop;
  PlyProperty **prop_list;

  /* find information about the element */
  elem = find_element (plyfile, elem_name);
  if (elem == NULL)
    return (NULL);

  *nelems = elem->num;
  *nprops = elem->nprops;

  /* make a copy of the element's property list */
  prop_list = (PlyProperty **) myalloc (sizeof (PlyProperty *) * elem->nprops);
  for (i = 0; i < elem->nprops; i++) {
    prop = (PlyProperty *) myalloc (sizeof (PlyProperty));
    copy_property (prop, elem->props[i]);
    prop_list[i] = prop;
  }

  /* return this duplicate property list */
  return (prop_list);
}


/******************************************************************************
Specify which properties of an element are to be returned.  This should be
called before a call to the routine get_element_ply().

Entry:
  plyfile   - file identifier
  elem_name - which element we're talking about
  nprops    - number of properties
  prop_list - list of properties
******************************************************************************/

void get_element_setup_ply(
  PlyFile *plyfile,
  char *elem_name,
  int nprops,
  PlyProperty *prop_list
)
{
  int i;
  PlyElement *elem;
  PlyProperty *prop;
  int index;

  /* find information about the element */
  elem = find_element (plyfile, elem_name);
  plyfile->which_elem = elem;

  /* deposit the property information into the element's description */
  for (i = 0; i < nprops; i++) {

    /* look for actual property */
    prop = find_property (elem, prop_list[i].name, &index);
    if (prop == NULL) {
      fprintf (stderr, "Warning:  Can't find property '%s' in element '%s'\n",
               prop_list[i].name, elem_name);
      continue;
    }

    /* store its description */
    prop->internal_type = prop_list[i].internal_type;
    prop->offset = prop_list[i].offset;
    prop->count_internal = prop_list[i].count_internal;
    prop->count_offset = prop_list[i].count_offset;

    /* specify that the user wants this property */
    elem->store_prop[index] = STORE_PROP;
  }
}


/******************************************************************************
Specify a property of an element that is to be returned.  This should be
called (usually multiple times) before a call to the routine ply_get_element().
This routine should be used in preference to the less flexible old routine
called ply_get_element_setup().

Entry:
  plyfile   - file identifier
  elem_name - which element we're talking about
  prop      - property to add to those that will be returned
******************************************************************************/

void ply_get_property(
  PlyFile *plyfile,
  char *elem_name,
  PlyProperty *prop
)
{
  PlyElement *elem;
  PlyProperty *prop_ptr;
  int index;

  /* find information about the element */
  elem = find_element (plyfile, elem_name);
  plyfile->which_elem = elem;

  /* deposit the property information into the element's description */

  prop_ptr = find_property (elem, prop->name, &index);
  if (prop_ptr == NULL) {
    fprintf (stderr, "Warning:  Can't find property '%s' in element '%s'\n",
             prop->name, elem_name);
    return;
  }
  prop_ptr->internal_type  = prop->internal_type;
  prop_ptr->offset         = prop->offset;
  prop_ptr->count_internal = prop->count_internal;
  prop_ptr->count_offset   = prop->count_offset;

  /* specify that the user wants this property */
  elem->store_prop[index] = STORE_PROP;
}


/******************************************************************************
Read one element from the file.  This routine assumes that we're reading
the type of element specified in the last call to the routine
ply_get_element_setup().

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to location where the element information should be put
******************************************************************************/

void ply_get_element(PlyFile *plyfile, void *elem_ptr)
{
  if (plyfile->file_type == PLY_ASCII)
    ascii_get_element (plyfile, (char *) elem_ptr);
  else
    binary_get_element (plyfile, (char *) elem_ptr);
}


/******************************************************************************
Extract the comments from the header information of a PLY file.

Entry:
  plyfile - file identifier

Exit:
  num_comments - number of comments returned
  returns a pointer to a list of comments
******************************************************************************/

char **get_comments_ply(PlyFile *plyfile, int *num_comments)
{
  *num_comments = plyfile->num_comments;
  return (plyfile->comments);
}


/******************************************************************************
Extract the object information (arbitrary text) from the header information
of a PLY file.

Entry:
  plyfile - file identifier

Exit:
  num_obj_info - number of lines of text information returned
  returns a pointer to a list of object info lines
******************************************************************************/

char **get_obj_info_ply(PlyFile *plyfile, int *num_obj_info)
{
  *num_obj_info = plyfile->num_obj_info;
  return (plyfile->obj_info);
}


/******************************************************************************
Make ready for "other" properties of an element-- those properties that
the user has not explicitly asked for, but that are to be stashed away
in a special structure to be carried along with the element's other
information.

Entry:
  plyfile - file identifier
  elem    - element for which we want to save away other properties
******************************************************************************/

void setup_other_props(PlyFile *plyfile, PlyElement *elem)
{
  int i;
  PlyProperty *prop;
  int size = 0;
  int type_size;

  /* Examine each property in decreasing order of size. */
  /* We do this so that all data types will be aligned by */
  /* word, half-word, or whatever within the structure. */

  for (type_size = 8; type_size > 0; type_size /= 2) {

    /* add up the space taken by each property, and save this information */
    /* away in the property descriptor */

    for (i = 0; i < elem->nprops; i++) {

      /* don't bother with properties we've been asked to store explicitly */
      if (elem->store_prop[i])
        continue;

      prop = elem->props[i];

      /* internal types will be same as external */
      prop->internal_type = prop->external_type;
      prop->count_internal = prop->count_external;

      /* list case */
      if (prop->is_list == PLY_LIST) {

        /* pointer to list */
        if (type_size == sizeof (void *)) {
          prop->offset = size;
          size += sizeof (void *);    /* always use size of a pointer here */
        }

        /* count of number of list elements */
        if (type_size == ply_type_size[prop->count_external]) {
          prop->count_offset = size;
          size += ply_type_size[prop->count_external];
        }
      }
      /* string */
      else if (prop->is_list == PLY_STRING) {
        /* pointer to string */
        if (type_size == sizeof (char *)) {
          prop->offset = size;
          size += sizeof (char *);
        }
      }
      /* scalar */
      else if (type_size == ply_type_size[prop->external_type]) {
        prop->offset = size;
        size += ply_type_size[prop->external_type];
      }
    }

  }

  /* save the size for the other_props structure */
  elem->other_size = size;
}


/******************************************************************************
Specify that we want the "other" properties of an element to be tucked
away within the user's structure.

Entry:
  plyfile - file identifier
  elem    - the element that we want to store other_props in
  offset  - offset to where other_props will be stored inside user's structure

Exit:
  returns pointer to structure containing description of other_props
******************************************************************************/

static PlyOtherProp *get_other_properties(
  PlyFile *plyfile,
  PlyElement *elem,
  int offset
)
{
  int i;
  PlyOtherProp *other;
  PlyProperty *prop;
  int nprops;

  /* remember that this is the "current" element */
  plyfile->which_elem = elem;

  /* save the offset to where to store the other_props */
  elem->other_offset = offset;

  /* place the appropriate pointers, etc. in the element's property list */
  setup_other_props (plyfile, elem);

  /* create structure for describing other_props */
  other = (PlyOtherProp *) myalloc (sizeof (PlyOtherProp));
  other->name = strdup (elem->name);
#if 0
  if (elem->other_offset == NO_OTHER_PROPS) {
    other->size = 0;
    other->props = NULL;
    other->nprops = 0;
    return (other);
  }
#endif
  other->size = elem->other_size;
  other->props = (PlyProperty **) myalloc (sizeof(PlyProperty) * elem->nprops);
  
  /* save descriptions of each "other" property */
  nprops = 0;
  for (i = 0; i < elem->nprops; i++) {
    if (elem->store_prop[i])
      continue;
    prop = (PlyProperty *) myalloc (sizeof (PlyProperty));
    copy_property (prop, elem->props[i]);
    other->props[nprops] = prop;
    nprops++;
  }
  other->nprops = nprops;

  /* set other_offset pointer appropriately if there are NO other properties */
  if (other->nprops == 0) {
    elem->other_offset = NO_OTHER_PROPS;
  }
 
  /* return structure */
  return (other);
}


/******************************************************************************
Specify that we want the "other" properties of an element to be tucked
away within the user's structure.  The user needn't be concerned for how
these properties are stored.

Entry:
  plyfile   - file identifier
  elem_name - name of element that we want to store other_props in
  offset    - offset to where other_props will be stored inside user's structure

Exit:
  returns pointer to structure containing description of other_props
******************************************************************************/

PlyOtherProp *ply_get_other_properties(
  PlyFile *plyfile,
  char *elem_name,
  int offset
)
{
  PlyElement *elem;
  PlyOtherProp *other;

  /* find information about the element */
  elem = find_element (plyfile, elem_name);
  if (elem == NULL) {
    fprintf (stderr, "ply_get_other_properties: Can't find element '%s'\n",
             elem_name);
    return (NULL);
  }

  other = get_other_properties (plyfile, elem, offset);
  return (other);
}




/*************************/
/*  Other Element Stuff  */
/*************************/





/******************************************************************************
Grab all the data for the current element that a user does not want to
explicitly read in.  Stores this in the PLY object's data structure.

Entry:
  plyfile - pointer to file

Exit:
  returns pointer to ALL the "other" element data for this PLY file
******************************************************************************/

PlyOtherElems *get_other_element_ply (PlyFile *plyfile)
{
  int i;
  PlyElement *elem;
  char *elem_name;
  int elem_count;
  PlyOtherElems *other_elems;
  OtherElem *other;

  elem = plyfile->which_elem;
  elem_name = elem->name;
  elem_count = elem->num;

  /* create room for the new "other" element, initializing the */
  /* other data structure if necessary */

  if (plyfile->other_elems == NULL) {
    plyfile->other_elems = (PlyOtherElems *) myalloc (sizeof (PlyOtherElems));
    other_elems = plyfile->other_elems;
    other_elems->other_list = (OtherElem *) myalloc (sizeof (OtherElem));
    other = &(other_elems->other_list[0]);
    other_elems->num_elems = 1;
  }
  else {
    other_elems = plyfile->other_elems;
    other_elems->other_list = (OtherElem *) realloc (other_elems->other_list,
                              sizeof (OtherElem) * other_elems->num_elems + 1);
    other = &(other_elems->other_list[other_elems->num_elems]);
    other_elems->num_elems++;
  }

  /* count of element instances in file */
  other->elem_count = elem_count;

  /* save name of element */
  other->elem_name = strdup (elem_name);

  /* create a list to hold all the current elements */
  other->other_data = (OtherData **)
                  malloc (sizeof (OtherData *) * other->elem_count);

  /* set up for getting elements */
  other->other_props = ply_get_other_properties (plyfile, elem_name,
                         offsetof(OtherData,other_props));

  /* grab all these elements */
  for (i = 0; i < other->elem_count; i++) {
    /* grab and element from the file */
    other->other_data[i] = (OtherData *) malloc (sizeof (OtherData));
    ply_get_element (plyfile, (void *) other->other_data[i]);
  }

  /* return pointer to the other elements data */
  return (other_elems);
}


/******************************************************************************
Write out the "other" elements specified for this PLY file.

Entry:
  plyfile - pointer to PLY file to write out other elements for
******************************************************************************/

void put_other_elements_ply (PlyFile *plyfile)
{
  int i,j;
  OtherElem *other;

  /* make sure we have other elements to write */
  if (plyfile->other_elems == NULL)
    return;

  /* write out the data for each "other" element */

  for (i = 0; i < plyfile->other_elems->num_elems; i++) {

    other = &(plyfile->other_elems->other_list[i]);
    put_element_setup_ply (plyfile, other->elem_name);

    /* write out each instance of the current element */
    for (j = 0; j < other->elem_count; j++)
      put_element_ply (plyfile, (void *) other->other_data[j]);
  }
}


/******************************************************************************
Free up storage used by an "other" elements data structure.

Entry:
  other_elems - data structure to free up
******************************************************************************/

void free_other_elements_ply (PlyOtherElems *other_elems)
{

}



/*******************/
/*  Miscellaneous  */
/*******************/



/******************************************************************************
Close a PLY file.

Entry:
  plyfile - identifier of file to close
******************************************************************************/

void ply_close(PlyFile *plyfile)
{
  fclose (plyfile->fp);

  /* free up memory associated with the PLY file */
  free (plyfile);
}


/******************************************************************************
Get version number and file type of a PlyFile.

Entry:
  ply - pointer to PLY file

Exit:
  version - version of the file
  file_type - PLY_ASCII, PLY_BINARY_BE, or PLY_BINARY_LE
******************************************************************************/

void get_info_ply(PlyFile *ply, float *version, int *file_type)
{
  if (ply == NULL)
    return;

  *version = ply->version;
  *file_type = ply->file_type;
}


/******************************************************************************
Compare two strings.  Returns 1 if they are the same, 0 if not.
******************************************************************************/

int equal_strings(char *s1, char *s2)
{
  while (*s1 && *s2)
    if (*s1++ != *s2++)
      return (0);

  if (*s1 != *s2)
    return (0);
  else
    return (1);
}


/******************************************************************************
Re-create the command line that was used to invoke this program.

Entry:
  argc - number of words in argv
  argv - array of words in command line
******************************************************************************/

char *recreate_command_line (int argc, char *argv[])
{
  int i;
  char *line;
  int len = 0;

  /* count total number of characters needed, including separating spaces */
  for (i = 0; i < argc; i++)
    len += strlen(argv[i]) + 1;

  /* create empty line */
  line = (char *) malloc (sizeof(char) * len);
  line[0] = '\0';

  /* repeatedly append argv */
  for (i = 0; i < argc; i++) {
    strcat (line, argv[i]);
    if (i != argc - 1)
      strcat (line, " ");
  }

  return (line);
}


/******************************************************************************
Find an element from the element list of a given PLY object.

Entry:
  plyfile - file id for PLY file
  element - name of element we're looking for

Exit:
  returns the element, or NULL if not found
******************************************************************************/

PlyElement *find_element(PlyFile *plyfile, char *element)
{
  int i;

  for (i = 0; i < plyfile->num_elem_types; i++)
    if (equal_strings (element, plyfile->elems[i]->name))
      return (plyfile->elems[i]);

  return (NULL);
}


/******************************************************************************
Find a property in the list of properties of a given element.

Entry:
  elem      - pointer to element in which we want to find the property
  prop_name - name of property to find

Exit:
  index - index to position in list
  returns a pointer to the property, or NULL if not found
******************************************************************************/

PlyProperty *find_property(PlyElement *elem, char *prop_name, int *index)
{
  int i;

  for (i = 0; i < elem->nprops; i++)
    if (equal_strings (prop_name, elem->props[i]->name)) {
      *index = i;
      return (elem->props[i]);
    }

  *index = -1;
  return (NULL);
}


/******************************************************************************
Read an element from an ascii file.

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to element
******************************************************************************/

void ascii_get_element(PlyFile *plyfile, char *elem_ptr)
{
  int j,k;
  PlyElement *elem;
  PlyProperty *prop;
  char **words;
  int nwords;
  int which_word;
  char *elem_data,*item = NULL;
  char *item_ptr;
  int item_size;
  int int_val;
  unsigned int uint_val;
  double double_val;
  int list_count;
  int store_it;
  char **store_array;
  char *orig_line;
  char *other_data = NULL;
  int other_flag;

  /* the kind of element we're reading currently */
  elem = plyfile->which_elem;

  /* do we need to setup for other_props? */

  if (elem->other_offset != NO_OTHER_PROPS) {
    char **ptr;
    other_flag = 1;
    /* make room for other_props */
    other_data = (char *) myalloc (elem->other_size);
    /* store pointer in user's structure to the other_props */
    ptr = (char **) (elem_ptr + elem->other_offset);
    *ptr = other_data;
  }
  else
    other_flag = 0;

  /* read in the element */

  words = get_words (plyfile->fp, &nwords, &orig_line);
  if (words == NULL) {
    fprintf (stderr, "ply_get_element: unexpected end of file\n");
    exit (-1);
  }

  which_word = 0;

  for (j = 0; j < elem->nprops; j++) {

    prop = elem->props[j];
    store_it = (elem->store_prop[j] | other_flag);

    /* store either in the user's structure or in other_props */
    if (elem->store_prop[j])
      elem_data = elem_ptr;
    else
      elem_data = other_data;

    if (prop->is_list == PLY_LIST) {       /* a list */

      /* get and store the number of items in the list */
      get_ascii_item (words[which_word++], prop->count_external,
                      &int_val, &uint_val, &double_val);
      if (store_it) {
        item = elem_data + prop->count_offset;
        store_item(item, prop->count_internal, int_val, uint_val, double_val);
      }

      /* allocate space for an array of items and store a ptr to the array */
      list_count = int_val;
      item_size = ply_type_size[prop->internal_type];
      store_array = (char **) (elem_data + prop->offset);

      if (list_count == 0) {
        if (store_it)
          *store_array = NULL;
      }
      else {
        if (store_it) {
          item_ptr = (char *) myalloc (sizeof (char) * item_size * list_count);
          item = item_ptr;
          *store_array = item_ptr;
        }

        /* read items and store them into the array */
        for (k = 0; k < list_count; k++) {
          get_ascii_item (words[which_word++], prop->external_type,
                          &int_val, &uint_val, &double_val);
          if (store_it) {
            store_item (item, prop->internal_type,
                        int_val, uint_val, double_val);
            item += item_size;
          }
        }
      }

    }
    else if (prop->is_list == PLY_STRING) {   /* a string */
      if (store_it) {
	char *str;
	char **str_ptr;
	str = strdup (words[which_word++]);
        item = elem_data + prop->offset;
	str_ptr = (char **) item;
	*str_ptr = str;
      }
      else {
        which_word++;
      }
    }
    else {                     /* a scalar */
      get_ascii_item (words[which_word++], prop->external_type,
                      &int_val, &uint_val, &double_val);
      if (store_it) {
        item = elem_data + prop->offset;
        store_item (item, prop->internal_type, int_val, uint_val, double_val);
      }
    }

  }

  free (words);
}


/******************************************************************************
Read an element from a binary file.

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to an element
******************************************************************************/

void binary_get_element(PlyFile *plyfile, char *elem_ptr)
{
  int j,k;
  PlyElement *elem;
  PlyProperty *prop;
  FILE *fp = plyfile->fp;
  char *elem_data;
  char *item = NULL;
  char *item_ptr;
  int item_size;
  int int_val;
  unsigned int uint_val;
  double double_val;
  int list_count;
  int store_it;
  char **store_array;
  char *other_data = NULL;
  int other_flag;

  /* the kind of element we're reading currently */
  elem = plyfile->which_elem;

  /* do we need to setup for other_props? */

  if (elem->other_offset != NO_OTHER_PROPS) {
    char **ptr;
    other_flag = 1;
    /* make room for other_props */
    other_data = (char *) myalloc (elem->other_size);
    /* store pointer in user's structure to the other_props */
    ptr = (char **) (elem_ptr + elem->other_offset);
    *ptr = other_data;
  }
  else
    other_flag = 0;

  /* read in a number of elements */

  for (j = 0; j < elem->nprops; j++) {

    prop = elem->props[j];
    store_it = (elem->store_prop[j] | other_flag);

    /* store either in the user's structure or in other_props */
    if (elem->store_prop[j])
      elem_data = elem_ptr;
    else
      elem_data = other_data;

    if (prop->is_list == PLY_LIST) {          /* list */

      /* get and store the number of items in the list */
      get_binary_item (fp, prop->count_external,
                      &int_val, &uint_val, &double_val);
      if (store_it) {
        item = elem_data + prop->count_offset;
        store_item(item, prop->count_internal, int_val, uint_val, double_val);
      }

      /* allocate space for an array of items and store a ptr to the array */
      list_count = int_val;
      item_size = ply_type_size[prop->internal_type];
      store_array = (char **) (elem_data + prop->offset);
      if (list_count == 0) {
        if (store_it)
          *store_array = NULL;
      }
      else {
        if (store_it) {
          item_ptr = (char *) myalloc (sizeof (char) * item_size * list_count);
          item = item_ptr;
          *store_array = item_ptr;
        }

        /* read items and store them into the array */
        for (k = 0; k < list_count; k++) {
          get_binary_item (fp, prop->external_type,
                          &int_val, &uint_val, &double_val);
          if (store_it) {
            store_item (item, prop->internal_type,
                        int_val, uint_val, double_val);
            item += item_size;
          }
        }
      }

    }
    else if (prop->is_list == PLY_STRING) {     /* string */
      int len;
      char *str;
      fread (&len, sizeof(int), 1, fp);
      str = (char *) myalloc (len);
      fread (str, len, 1, fp);
      if (store_it) {
	char **str_ptr;
        item = elem_data + prop->offset;
	str_ptr = (char **) item;
	*str_ptr = str;
      }
    }
    else {                                      /* scalar */
      get_binary_item (fp, prop->external_type,
                      &int_val, &uint_val, &double_val);
      if (store_it) {
        item = elem_data + prop->offset;
        store_item (item, prop->internal_type, int_val, uint_val, double_val);
      }
    }

  }
}


/******************************************************************************
Write to a file the word that represents a PLY data type.

Entry:
  fp   - file pointer
  code - code for type
******************************************************************************/

void write_scalar_type (FILE *fp, int code)
{
  /* make sure this is a valid code */

  if (code <= StartType || code >= EndType) {
    fprintf (stderr, "write_scalar_type: bad data code = %d\n", code);
    exit (-1);
  }

  /* write the code to a file */

  fprintf (fp, "%s", type_names[code]);
}


/******************************************************************************
Get a text line from a file and break it up into words.

IMPORTANT: The calling routine should call "free" on the returned pointer once
finished with it.

Entry:
  fp - file to read from

Exit:
  nwords    - number of words returned
  orig_line - the original line of characters
  returns a list of words from the line, or NULL if end-of-file
******************************************************************************/

char **get_words(FILE *fp, int *nwords, char **orig_line)
{
#define BIG_STRING 4096
  static char str[BIG_STRING];
  static char str_copy[BIG_STRING];
  char **words;
  int max_words = 10;
  int num_words = 0;
  char *ptr,*ptr2;
  char *result;

  words = (char **) myalloc (sizeof (char *) * max_words);

  /* read in a line */
  result = fgets (str, BIG_STRING, fp);
  if (result == NULL) {
    *nwords = 0;
    *orig_line = NULL;
    return (NULL);
  }

  /* convert line-feed and tabs into spaces */
  /* (this guarentees that there will be a space before the */
  /*  null character at the end of the string) */

  str[BIG_STRING-2] = ' ';
  str[BIG_STRING-1] = '\0';

  for (ptr = str, ptr2 = str_copy; *ptr != '\0'; ptr++, ptr2++) {
    *ptr2 = *ptr;
    if (*ptr == '\t') {
      *ptr = ' ';
      *ptr2 = ' ';
    }
    else if (*ptr == '\n') {
      *ptr = ' ';
      *ptr2 = '\0';
      break;
    }
  }

  /* find the words in the line */

  ptr = str;
  while (*ptr != '\0') {

    /* jump over leading spaces */
    while (*ptr == ' ')
      ptr++;

    /* break if we reach the end */
    if (*ptr == '\0')
      break;

    /* allocate more room for words if necessary */
    if (num_words >= max_words) {
      max_words += 10;
      words = (char **) realloc (words, sizeof (char *) * max_words);
    }

    if (*ptr == '\"') {  /* a quote indidicates that we have a string */

      /* skip over leading quote */
      ptr++;

      /* save pointer to beginning of word */
      words[num_words++] = ptr;

      /* find trailing quote or end of line */
      while (*ptr != '\"' && *ptr != '\0')
        ptr++;

      /* replace quote with a null character to mark the end of the word */
      /* if we are not already at the end of the line */
      if (*ptr != '\0')
	*ptr++ = '\0';
    }
    else {               /* non-string */

      /* save pointer to beginning of word */
      words[num_words++] = ptr;

      /* jump over non-spaces */
      while (*ptr != ' ')
	ptr++;

      /* place a null character here to mark the end of the word */
      *ptr++ = '\0';
    }
  }

  /* return the list of words */
  *nwords = num_words;
  *orig_line = str_copy;
  return (words);
}


/******************************************************************************
Return the value of an item, given a pointer to it and its type.

Entry:
  item - pointer to item
  type - data type that "item" points to

Exit:
  returns a double-precision float that contains the value of the item
******************************************************************************/

double get_item_value(char *item, int type)
{
  unsigned char *puchar;
  char *pchar;
  short int *pshort;
  unsigned short int *pushort;
  int *pint;
  unsigned int *puint;
  float *pfloat;
  double *pdouble;
  int int_value;
  unsigned int uint_value;
  double double_value;

  switch (type) {
    case Int8:
      pchar = (char *) item;
      int_value = *pchar;
      return ((double) int_value);
    case Uint8:
      puchar = (unsigned char *) item;
      int_value = *puchar;
      return ((double) int_value);
    case Int16:
      pshort = (short int *) item;
      int_value = *pshort;
      return ((double) int_value);
    case Uint16:
      pushort = (unsigned short int *) item;
      int_value = *pushort;
      return ((double) int_value);
    case Int32:
      pint = (int *) item;
      int_value = *pint;
      return ((double) int_value);
    case Uint32:
      puint = (unsigned int *) item;
      uint_value = *puint;
      return ((double) uint_value);
    case Float32:
      pfloat = (float *) item;
      double_value = *pfloat;
      return (double_value);
    case Float64:
      pdouble = (double *) item;
      double_value = *pdouble;
      return (double_value);
    default:
      fprintf (stderr, "get_item_value: bad type = %d\n", type);
      exit (-1);
  }

  return (0.0);  /* never actually gets here */
}


/******************************************************************************
Write out an item to a file as raw binary bytes.

Entry:
  fp         - file to write to
  int_val    - integer version of item
  uint_val   - unsigned integer version of item
  double_val - double-precision float version of item
  type       - data type to write out
******************************************************************************/

void write_binary_item(
  FILE *fp,
  int int_val,
  unsigned int uint_val,
  double double_val,
  int type
)
{
  unsigned char uchar_val;
  char char_val;
  unsigned short ushort_val;
  short short_val;
  float float_val;

  switch (type) {
    case Int8:
      char_val = int_val;
      fwrite (&char_val, 1, 1, fp);
      break;
    case Int16:
      short_val = int_val;
      fwrite (&short_val, 2, 1, fp);
      break;
    case Int32:
      fwrite (&int_val, 4, 1, fp);
      break;
    case Uint8:
      uchar_val = uint_val;
      fwrite (&uchar_val, 1, 1, fp);
      break;
    case Uint16:
      ushort_val = uint_val;
      fwrite (&ushort_val, 2, 1, fp);
      break;
    case Uint32:
      fwrite (&uint_val, 4, 1, fp);
      break;
    case Float32:
      float_val = double_val;
      fwrite (&float_val, 4, 1, fp);
      break;
    case Float64:
      fwrite (&double_val, 8, 1, fp);
      break;
    default:
      fprintf (stderr, "write_binary_item: bad type = %d\n", type);
      exit (-1);
  }
}


/******************************************************************************
Write out an item to a file as ascii characters.

Entry:
  fp         - file to write to
  int_val    - integer version of item
  uint_val   - unsigned integer version of item
  double_val - double-precision float version of item
  type       - data type to write out
******************************************************************************/

void write_ascii_item(
  FILE *fp,
  int int_val,
  unsigned int uint_val,
  double double_val,
  int type
)
{
  switch (type) {
    case Int8:
    case Int16:
    case Int32:
      fprintf (fp, "%d ", int_val);
      break;
    case Uint8:
    case Uint16:
    case Uint32:
      fprintf (fp, "%u ", uint_val);
      break;
    case Float32:
    case Float64:
      fprintf (fp, "%g ", double_val);
      break;
    default:
      fprintf (stderr, "write_ascii_item: bad type = %d\n", type);
      exit (-1);
  }
}


/******************************************************************************
Get the value of an item that is in memory, and place the result
into an integer, an unsigned integer and a double.

Entry:
  ptr  - pointer to the item
  type - data type supposedly in the item

Exit:
  int_val    - integer value
  uint_val   - unsigned integer value
  double_val - double-precision floating point value
******************************************************************************/

void get_stored_item(
  void *ptr,
  int type,
  int *int_val,
  unsigned int *uint_val,
  double *double_val
)
{
  switch (type) {
    case Int8:
      *int_val = *((char *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case Uint8:
      *uint_val = *((unsigned char *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case Int16:
      *int_val = *((short int *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case Uint16:
      *uint_val = *((unsigned short int *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case Int32:
      *int_val = *((int *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case Uint32:
      *uint_val = *((unsigned int *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case Float32:
      *double_val = *((float *) ptr);
      *int_val = *double_val;
      *uint_val = *double_val;
      break;
    case Float64:
      *double_val = *((double *) ptr);
      *int_val = *double_val;
      *uint_val = *double_val;
      break;
    default:
      fprintf (stderr, "get_stored_item: bad type = %d\n", type);
      exit (-1);
  }
}


/******************************************************************************
Get the value of an item from a binary file, and place the result
into an integer, an unsigned integer and a double.

Entry:
  fp   - file to get item from
  type - data type supposedly in the word

Exit:
  int_val    - integer value
  uint_val   - unsigned integer value
  double_val - double-precision floating point value
******************************************************************************/

void get_binary_item(
  FILE *fp,
  int type,
  int *int_val,
  unsigned int *uint_val,
  double *double_val
)
{
  char c[8];
  void *ptr;

  ptr = (void *) c;

  switch (type) {
    case Int8:
      fread (ptr, 1, 1, fp);
      *int_val = *((char *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case Uint8:
      fread (ptr, 1, 1, fp);
      *uint_val = *((unsigned char *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case Int16:
      fread (ptr, 2, 1, fp);
      *int_val = *((short int *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case Uint16:
      fread (ptr, 2, 1, fp);
      *uint_val = *((unsigned short int *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case Int32:
      fread (ptr, 4, 1, fp);
      *int_val = *((int *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case Uint32:
      fread (ptr, 4, 1, fp);
      *uint_val = *((unsigned int *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case Float32:
      fread (ptr, 4, 1, fp);
      *double_val = *((float *) ptr);
      *int_val = *double_val;
      *uint_val = *double_val;
      break;
    case Float64:
      fread (ptr, 8, 1, fp);
      *double_val = *((double *) ptr);
      *int_val = *double_val;
      *uint_val = *double_val;
      break;
    default:
      fprintf (stderr, "get_binary_item: bad type = %d\n", type);
      exit (-1);
  }
}


/******************************************************************************
Extract the value of an item from an ascii word, and place the result
into an integer, an unsigned integer and a double.

Entry:
  word - word to extract value from
  type - data type supposedly in the word

Exit:
  int_val    - integer value
  uint_val   - unsigned integer value
  double_val - double-precision floating point value
******************************************************************************/

void get_ascii_item(
  char *word,
  int type,
  int *int_val,
  unsigned int *uint_val,
  double *double_val
)
{
  switch (type) {
    case Int8:
    case Uint8:
    case Int16:
    case Uint16:
    case Int32:
      *int_val = atoi (word);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;

    case Uint32:
      *uint_val = strtoul (word, (char **) NULL, 10);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;

    case Float32:
    case Float64:
      *double_val = atof (word);
      *int_val = (int) *double_val;
      *uint_val = (unsigned int) *double_val;
      break;

    default:
      fprintf (stderr, "get_ascii_item: bad type = %d\n", type);
      exit (-1);
  }
}


/******************************************************************************
Store a value into a place being pointed to, guided by a data type.

Entry:
  item       - place to store value
  type       - data type
  int_val    - integer version of value
  uint_val   - unsigned integer version of value
  double_val - double version of value

Exit:
  item - pointer to stored value
******************************************************************************/

void store_item (
  char *item,
  int type,
  int int_val,
  unsigned int uint_val,
  double double_val
)
{
  unsigned char *puchar;
  short int *pshort;
  unsigned short int *pushort;
  int *pint;
  unsigned int *puint;
  float *pfloat;
  double *pdouble;

  switch (type) {
    case Int8:
      *item = int_val;
      break;
    case Uint8:
      puchar = (unsigned char *) item;
      *puchar = uint_val;
      break;
    case Int16:
      pshort = (short *) item;
      *pshort = int_val;
      break;
    case Uint16:
      pushort = (unsigned short *) item;
      *pushort = uint_val;
      break;
    case Int32:
      pint = (int *) item;
      *pint = int_val;
      break;
    case Uint32:
      puint = (unsigned int *) item;
      *puint = uint_val;
      break;
    case Float32:
      pfloat = (float *) item;
      *pfloat = double_val;
      break;
    case Float64:
      pdouble = (double *) item;
      *pdouble = double_val;
      break;
    default:
      fprintf (stderr, "store_item: bad type = %d\n", type);
      exit (-1);
  }
}


/******************************************************************************
Add an element to a PLY file descriptor.

Entry:
  plyfile - PLY file descriptor
  words   - list of words describing the element
  nwords  - number of words in the list
******************************************************************************/

void add_element (PlyFile *plyfile, char **words, int nwords)
{
  PlyElement *elem;

  /* create the new element */
  elem = (PlyElement *) myalloc (sizeof (PlyElement));
  elem->name = strdup (words[1]);
  elem->num = atoi (words[2]);
  elem->nprops = 0;

  /* make room for new element in the object's list of elements */
  if (plyfile->num_elem_types == 0)
    plyfile->elems = (PlyElement **) myalloc (sizeof (PlyElement *));
  else
    plyfile->elems = (PlyElement **) realloc (plyfile->elems,
                     sizeof (PlyElement *) * (plyfile->num_elem_types + 1));

  /* add the new element to the object's list */
  plyfile->elems[plyfile->num_elem_types] = elem;
  plyfile->num_elem_types++;
}


/******************************************************************************
Return the type of a property, given the name of the property.

Entry:
  name - name of property type

Exit:
  returns integer code for property, or 0 if not found
******************************************************************************/

int get_prop_type(char *type_name)
{
  int i;

  /* try to match the type name */
  for (i = StartType + 1; i < EndType; i++)
    if (equal_strings (type_name, type_names[i]))
      return (i);

  /* see if we can match an old type name */
  for (i = StartType + 1; i < EndType; i++)
    if (equal_strings (type_name, old_type_names[i]))
      return (i);

  /* if we get here, we didn't find the type */
  return (0);
}


/******************************************************************************
Add a property to a PLY file descriptor.

Entry:
  plyfile - PLY file descriptor
  words   - list of words describing the property
  nwords  - number of words in the list
******************************************************************************/

void add_property (PlyFile *plyfile, char **words, int nwords)
{
  PlyProperty *prop;
  PlyElement *elem;

  /* create the new property */

  prop = (PlyProperty *) myalloc (sizeof (PlyProperty));

  if (equal_strings (words[1], "list")) {          /* list */
    prop->count_external = get_prop_type (words[2]);
    prop->external_type = get_prop_type (words[3]);
    prop->name = strdup (words[4]);
    prop->is_list = PLY_LIST;
  }
  else if (equal_strings (words[1], "string")) {   /* string */
    prop->count_external = Int8;
    prop->external_type = Int8;
    prop->name = strdup (words[2]);
    prop->is_list = PLY_STRING;
  }
  else {                                           /* scalar */
    prop->external_type = get_prop_type (words[1]);
    prop->name = strdup (words[2]);
    prop->is_list = PLY_SCALAR;
  }

  /* add this property to the list of properties of the current element */

  elem = plyfile->elems[plyfile->num_elem_types - 1];

  if (elem->nprops == 0)
    elem->props = (PlyProperty **) myalloc (sizeof (PlyProperty *));
  else
    elem->props = (PlyProperty **) realloc (elem->props,
                  sizeof (PlyProperty *) * (elem->nprops + 1));

  elem->props[elem->nprops] = prop;
  elem->nprops++;
}


/******************************************************************************
Add a comment to a PLY file descriptor.

Entry:
  plyfile - PLY file descriptor
  line    - line containing comment
******************************************************************************/

void add_comment (PlyFile *plyfile, char *line)
{
  int i;

  /* skip over "comment" and leading spaces and tabs */
  i = 7;
  while (line[i] == ' ' || line[i] == '\t')
    i++;

  append_comment_ply (plyfile, &line[i]);
}


/******************************************************************************
Add a some object information to a PLY file descriptor.

Entry:
  plyfile - PLY file descriptor
  line    - line containing text info
******************************************************************************/

void add_obj_info (PlyFile *plyfile, char *line)
{
  int i;

  /* skip over "obj_info" and leading spaces and tabs */
  i = 8;
  while (line[i] == ' ' || line[i] == '\t')
    i++;

  append_obj_info_ply (plyfile, &line[i]);
}


/******************************************************************************
Copy a property.
******************************************************************************/

void copy_property(PlyProperty *dest, PlyProperty *src)
{
  dest->name = strdup (src->name);
  dest->external_type = src->external_type;
  dest->internal_type = src->internal_type;
  dest->offset = src->offset;

  dest->is_list = src->is_list;
  dest->count_external = src->count_external;
  dest->count_internal = src->count_internal;
  dest->count_offset = src->count_offset;
}


/******************************************************************************
Allocate some memory.

Entry:
  size  - amount of memory requested (in bytes)
  lnum  - line number from which memory was requested
  fname - file name from which memory was requested
******************************************************************************/

static char *my_alloc(int size, int lnum, char *fname)
{
  char *ptr;

  ptr = (char *) malloc (size);

  if (ptr == 0) {
    fprintf(stderr, "Memory allocation bombed on line %d in %s\n", lnum, fname);
  }

  return (ptr);
}


/**** NEW STUFF ****/
/**** NEW STUFF ****/
/**** NEW STUFF ****/
/**** NEW STUFF ****/



/******************************************************************************
Given a file pointer, get ready to read PLY data from the file.

Entry:
  fp - the given file pointer

Exit:
  nelems     - number of elements in object
  elem_names - list of element names
  returns a pointer to a PlyFile, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *read_ply(FILE *fp)
{
  PlyFile *ply;
  int num_elems;
  char **elem_names;

  ply = ply_read (fp, &num_elems, &elem_names);

  return (ply);
}


/******************************************************************************
Given a file pointer, get ready to write PLY data to the file.

Entry:
  fp         - the given file pointer
  nelems     - number of elements in object
  elem_names - list of element names
  file_type  - file type, either ascii or binary

Exit:
  returns a pointer to a PlyFile, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *write_ply(
  FILE *fp,
  int nelems,
  char **elem_names,
  int file_type
)
{
  PlyFile *ply;

  ply = ply_write (fp, nelems, elem_names, file_type);

  return (ply);
}


/******************************************************************************
Return a list of the names of the elements in a particular PLY file.

Entry:
  ply - PLY file whose element name list we want

Exit:
  num_elems  - the number of element names in the list
  returns the list of names
******************************************************************************/

char **get_element_list_ply(PlyFile *ply, int *num_elems)
{
  int i;
  char **elist;

  /* create the list of element names */

  elist = (char **) myalloc (sizeof (char *) * ply->num_elem_types);
  for (i = 0; i < ply->num_elem_types; i++)
    elist[i] = strdup (ply->elems[i]->name);

  /* return the number of elements and the list of element names */
  *num_elems = ply->num_elem_types;
  return (elist);
}


/******************************************************************************
Append a comment to a PLY file.

Entry:
  ply     - file to append comment to
  comment - the comment to append
******************************************************************************/

void append_comment_ply(PlyFile *ply, char *comment)
{
  /* (re)allocate space for new comment */
  if (ply->num_comments == 0)
    ply->comments = (char **) myalloc (sizeof (char *));
  else
    ply->comments = (char **) realloc (ply->comments,
		     sizeof (char *) * (ply->num_comments + 1));

  /* add comment to list */
  ply->comments[ply->num_comments] = strdup (comment);
  ply->num_comments++;
}


/******************************************************************************
Copy the comments from one PLY file to another.

Entry:
  out_ply - destination file to copy comments to
  in_ply  - the source of the comments
******************************************************************************/

void copy_comments_ply(PlyFile *out_ply, PlyFile *in_ply)
{
  int i;

  for (i = 0; i < in_ply->num_comments; i++)
    append_comment_ply (out_ply, in_ply->comments[i]);
}


/******************************************************************************
Append object information (arbitrary text) to a PLY file.

Entry:
  ply      - file to append object info to
  obj_info - the object info to append
******************************************************************************/

void append_obj_info_ply(PlyFile *ply, char *obj_info)
{
  /* (re)allocate space for new info */
  if (ply->num_obj_info == 0)
    ply->obj_info = (char **) myalloc (sizeof (char *));
  else
    ply->obj_info = (char **) realloc (ply->obj_info,
		     sizeof (char *) * (ply->num_obj_info + 1));

  /* add info to list */
  ply->obj_info[ply->num_obj_info] = strdup (obj_info);
  ply->num_obj_info++;
}


/******************************************************************************
Copy the object information from one PLY file to another.

Entry:
  out_ply - destination file to copy object information to
  in_ply  - the source of the object information
******************************************************************************/

void copy_obj_info_ply(PlyFile *out_ply, PlyFile *in_ply)
{
  int i;

  for (i = 0; i < in_ply->num_obj_info; i++)
    append_obj_info_ply (out_ply, in_ply->obj_info[i]);
}


/******************************************************************************
Close a PLY file.

Entry:
  plyfile - identifier of file to close
******************************************************************************/

void close_ply(PlyFile *plyfile)
{
  fclose (plyfile->fp);
}


/******************************************************************************
Free the memory used by a PLY file.

Entry:
  plyfile - identifier of file
******************************************************************************/

void free_ply(PlyFile *plyfile)
{
  /* free up memory associated with the PLY file */
  free (plyfile);
}


/******************************************************************************
Specify the index of the next element to be read in from a PLY file.

Entry:
  ply - file to read from
  index - index of the element to be read

Exit:
  elem_count - the number of elements in the file
  returns pointer to the name of this next element
******************************************************************************/

char *setup_element_read_ply (PlyFile *ply, int index, int *elem_count)
{
  PlyElement *elem;

  if (index < 0 || index > ply->num_elem_types) {
    fprintf (stderr, "Warning:  No element with index %d\n", index);
    return (0);
  }

  elem = ply->elems[index];

  /* set this to be the current element */
  ply->which_elem = elem;

  /* return the number of such elements in the file and the element's name */
  *elem_count = elem->num;
  return (elem->name);
}


/******************************************************************************
Read one element from the file.  This routine assumes that we're reading
the type of element specified in the last call to the routine
setup_element_read_ply().

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to location where the element information should be put
******************************************************************************/

void get_element_ply (PlyFile *plyfile, void *elem_ptr)
{
  if (plyfile->file_type == PLY_ASCII)
    ascii_get_element (plyfile, (char *) elem_ptr);
  else
    binary_get_element (plyfile, (char *) elem_ptr);
}


/******************************************************************************
Specify one of several properties of the current element that is to be
read from a file.  This should be called (usually multiple times) before a
call to the routine get_element_ply().

Entry:
  plyfile - file identifier
  prop    - property to add to those that will be returned
******************************************************************************/

void setup_property_ply(
  PlyFile *plyfile,
  PlyProperty *prop
)
{
  PlyElement *elem;
  PlyProperty *prop_ptr;
  int index;

  elem = plyfile->which_elem;

  /* deposit the property information into the element's description */

  prop_ptr = find_property (elem, prop->name, &index);
  if (prop_ptr == NULL) {
    fprintf (stderr, "Warning:  Can't find property '%s' in element '%s'\n",
             prop->name, elem->name);
    return;
  }
  prop_ptr->internal_type  = prop->internal_type;
  prop_ptr->offset         = prop->offset;
  prop_ptr->count_internal = prop->count_internal;
  prop_ptr->count_offset   = prop->count_offset;

  /* specify that the user wants this property */
  elem->store_prop[index] = STORE_PROP;
}


/******************************************************************************
Specify that we want the "other" properties of the current element to be tucked
away within the user's structure.

Entry:
  plyfile - file identifier
  offset  - offset to where other_props will be stored inside user's structure

Exit:
  returns pointer to structure containing description of other_props
******************************************************************************/

PlyOtherProp *get_other_properties_ply(
  PlyFile *plyfile,
  int offset
)
{
  PlyOtherProp *other;

  other = get_other_properties (plyfile, plyfile->which_elem, offset);
  return (other);
}


/******************************************************************************
Describe which element is to be written next and state how many of them will
be written.

Entry:
  plyfile   - file identifier
  elem_name - name of element that information is being described
  nelems    - number of elements of this type to be written
******************************************************************************/

void describe_element_ply(
  PlyFile *plyfile,
  char *elem_name,
  int nelems
)
{
  PlyElement *elem;

  /* look for appropriate element */
  elem = find_element (plyfile, elem_name);
  if (elem == NULL) {
    fprintf(stderr,"describe_element_ply: can't find element '%s'\n",elem_name);
    exit (-1);
  }

  elem->num = nelems;

  /* now this element is the current element */
  plyfile->which_elem = elem;
}


/******************************************************************************
Describe a property of an element.

Entry:
  plyfile   - file identifier
  prop      - the new property
******************************************************************************/

void describe_property_ply(
  PlyFile *plyfile,
  PlyProperty *prop
)
{
  PlyElement *elem;
  PlyProperty *elem_prop;

  elem = plyfile->which_elem;

  /* create room for new property */

  if (elem->nprops == 0) {
    elem->props = (PlyProperty **) myalloc (sizeof (PlyProperty *));
    elem->store_prop = (char *) myalloc (sizeof (char));
    elem->nprops = 1;
  }
  else {
    elem->nprops++;
    elem->props = (PlyProperty **)
                  realloc (elem->props, sizeof (PlyProperty *) * elem->nprops);
    elem->store_prop = (char *)
                  realloc (elem->store_prop, sizeof (char) * elem->nprops);
  }

  /* copy the new property */

  elem_prop = (PlyProperty *) myalloc (sizeof (PlyProperty));
  elem->props[elem->nprops - 1] = elem_prop;
  elem->store_prop[elem->nprops - 1] = NAMED_PROP;
  copy_property (elem_prop, prop);
}


/******************************************************************************
Describe what the "other" properties are that are to be stored, and where
they are in an element.
******************************************************************************/

void describe_other_properties_ply(
  PlyFile *plyfile,
  PlyOtherProp *other,
  int offset
)
{
  int i;
  PlyElement *elem;
  PlyProperty *prop;

  /* look for appropriate element */
  elem = find_element (plyfile, other->name);
  if (elem == NULL) {
    fprintf(stderr, "describe_other_properties_ply: can't find element '%s'\n",
            other->name);
    return;
  }

  /* create room for other properties */

  if (elem->nprops == 0) {
    elem->props = (PlyProperty **)
                  myalloc (sizeof (PlyProperty *) * other->nprops);
    elem->store_prop = (char *) myalloc (sizeof (char) * other->nprops);
    elem->nprops = 0;
  }
  else {
    int newsize;
    newsize = elem->nprops + other->nprops;
    elem->props = (PlyProperty **)
                  realloc (elem->props, sizeof (PlyProperty *) * newsize);
    elem->store_prop = (char *)
                  realloc (elem->store_prop, sizeof (char) * newsize);
  }

  /* copy the other properties */

  for (i = 0; i < other->nprops; i++) {
    prop = (PlyProperty *) myalloc (sizeof (PlyProperty));
    copy_property (prop, other->props[i]);
    elem->props[elem->nprops] = prop;
    elem->store_prop[elem->nprops] = OTHER_PROP;
    elem->nprops++;
  }

  /* save other info about other properties */
  elem->other_size = other->size;
  elem->other_offset = offset;
}


/******************************************************************************
Pass along a pointer to "other" elements that we want to save in a given
PLY file.  These other elements were presumably read from another PLY file.

Entry:
  plyfile     - file pointer in which to store this other element info
  other_elems - info about other elements that we want to store
******************************************************************************/

void describe_other_elements_ply (
  PlyFile *plyfile,
  PlyOtherElems *other_elems
)
{
  int i;
  OtherElem *other;

  /* ignore this call if there is no other element */
  if (other_elems == NULL)
    return;

  /* save pointer to this information */
  plyfile->other_elems = other_elems;

  /* describe the other properties of this element */

  for (i = 0; i < other_elems->num_elems; i++) {
    other = &(other_elems->other_list[i]);
    element_count_ply (plyfile, other->elem_name, other->elem_count);
    describe_other_properties_ply (plyfile, other->other_props,
                                   offsetof(OtherData,other_props));
  }
}



/**** Property Propagation Rules ****/


typedef struct RuleName {
  int code;
  char *name;
} RuleName;

RuleName rule_name_list[] = {
    { AVERAGE_RULE, "avg", },
    { RANDOM_RULE, "rnd", },
    { MINIMUM_RULE, "max", },
    { MAXIMUM_RULE, "min", },
    { MAJORITY_RULE, "major", },
    { SAME_RULE, "same", },
    { -1, "end_marker", },
};



/******************************************************************************
Initialize the property propagation rules for an element.  Default is to
use averaging (AVERAGE_RULE) for creating all new properties.

Entry:
  ply       - PLY object that this is for
  elem_name - name of the element that we're making the rules for

Exit:
  returns pointer to the default rules
******************************************************************************/

PlyPropRules *init_rule_ply (PlyFile *ply, char *elem_name)
{
  int i,j;
  PlyElement *elem;
  PlyPropRules *rules;
  PlyRuleList *list;
  int found_prop;

  elem = find_element (ply, elem_name);
  if (elem == NULL) {
    fprintf (stderr, "init_rule_ply: Can't find element '%s'\n", elem_name);
    exit (-1);
  }

  rules = (PlyPropRules *) myalloc (sizeof (PlyPropRules));
  rules->elem = elem;
  rules->rule_list = (int *) myalloc (sizeof(int) * elem->nprops);
  rules->max_props = 0;
  rules->nprops = 0;

  /* default is to use averaging rule */
  for (i = 0; i < elem->nprops; i++)
    rules->rule_list[i] = AVERAGE_RULE;

  /* see if there are other rules we should use */

  if (ply->rule_list == NULL)
    return (rules);

  /* try to match the element, property and rule name */

  for (list = ply->rule_list; list != NULL; list = list->next) {

    if (!equal_strings (list->element, elem->name))
      continue;

    found_prop = 0;

    for (i = 0; i < elem->nprops; i++)
      if (equal_strings (list->property, elem->props[i]->name)) {

        found_prop = 1;

        /* look for matching rule name */
        for (j = 0; rule_name_list[j].code != -1; j++)
          if (equal_strings (list->name, rule_name_list[j].name)) {
            rules->rule_list[i] = rule_name_list[j].code;
            break;
          }
      }

    if (!found_prop) {
      fprintf (stderr, "Can't find property '%s' for rule '%s'\n",
               list->property, list->name);
      continue;
    }
  }

  return (rules);
}


/******************************************************************************
Modify a property propagation rule.

Entry:
  rules - rules for the element
  prop_name - name of the property whose rule we're modifying
  rule_type - type of rule (MAXIMUM_RULE, MINIMUM_RULE, MAJORITY_RULE, etc.)
******************************************************************************/

void modify_rule_ply (PlyPropRules *rules, char *prop_name, int rule_type)
{
  int i;
  PlyElement *elem = rules->elem;

  /* find the property and modify its rule type */

  for (i = 0; i < elem->nprops; i++)
    if (equal_strings (elem->props[i]->name, prop_name)) {
      rules->rule_list[i] = rule_type;
      return;
    }

  /* we didn't find the property if we get here */
  fprintf (stderr, "modify_rule_ply: Can't find property '%s'\n", prop_name);
  exit (-1);
}


/******************************************************************************
Begin to create a set of properties from a set of propagation rules.

Entry:
  ply   - PLY object whose rules we're preparing to use
  rules - rules for the element
******************************************************************************/

void start_props_ply (PlyFile *ply, PlyPropRules *rules)
{
  /* save pointer to the rules in the PLY object */
  ply->current_rules = rules;

  /* get ready for new sets of properties to combine */
  rules->nprops = 0;
}


/******************************************************************************
Remember a set of properties and their weights for creating a new set of
properties.

Entry:
  weight      - weights for this set of properties
  other_props - the properties to use
******************************************************************************/

void weight_props_ply (PlyFile *ply, float weight, void *other_props)
{
  PlyPropRules *rules = ply->current_rules;

  /* allocate space for properties and weights, if necessary */
  if (rules->max_props == 0) {
    rules->max_props = 6;
    rules->props = (void **) myalloc (sizeof (void *) * rules->max_props);
    rules->weights = (float *) myalloc (sizeof (float) * rules->max_props);
  }
  if (rules->nprops == rules->max_props) {
    rules->max_props *= 2;
    rules->props = (void **) realloc (rules->props,
                   sizeof (void *) * rules->max_props);
    rules->weights = (float *) realloc (rules->weights,
                     sizeof (float) * rules->max_props);
  }

  /* remember these new properties and their weights */

  rules->props[rules->nprops] = other_props;
  rules->weights[rules->nprops] = weight;
  rules->nprops++;
}


/******************************************************************************
Return a pointer to a new set of properties that have been created using
a specified set of property combination rules and a given collection of
"other" properties.

Exit:
  returns a pointer to the new properties
******************************************************************************/

void *get_new_props_ply(PlyFile *ply)
{
  int i,j;
  static double *vals;
  static int max_vals = 0;
  PlyPropRules *rules = ply->current_rules;
  PlyElement *elem = rules->elem;
  PlyProperty *prop;
  char *data;
  char *new_data;
  void *ptr;
  int offset;
  int type;
  double double_val;
  int int_val;
  unsigned int uint_val;
  int random_pick;

  /* return NULL if we've got no "other" properties */
  if (elem->other_size == 0) {
    return (NULL);
  }

  /* create room for combined other properties */
  new_data = (char *) myalloc (sizeof (char) * elem->other_size);

  /* make sure there is enough room to store values we're to combine */

  if (max_vals == 0) {
    max_vals = rules->nprops;
    vals = (double *) myalloc (sizeof (double) * rules->nprops);
  }
  if (rules->nprops >= max_vals) {
    max_vals = rules->nprops;
    vals = (double *) realloc (vals, sizeof (double) * rules->nprops);
  }

  /* in case we need a random choice */
#ifdef WIN32
#define drand48(x) (rand(x)/(float)RAND_MAX)
#endif
  random_pick = (int) floor (rules->nprops * drand48());

  /* calculate the combination for each "other" property of the element */

  for (i = 0; i < elem->nprops; i++) {

    /* don't bother with properties we've been asked to store explicitly */
    if (elem->store_prop[i])
      continue;

    prop = elem->props[i];
    offset = prop->offset;
    type = prop->external_type;

    /* collect together all the values we're to combine */

    for (j = 0; j < rules->nprops; j++) {
      data = (char *) rules->props[j];
      ptr = (void *) (data + offset);
      get_stored_item ((void *) ptr, type, &int_val, &uint_val, &double_val);
      vals[j] = double_val;
    }

    /* calculate the combined value */

    switch (rules->rule_list[i]) {
      case AVERAGE_RULE: {
	double sum = 0;
	double weight_sum = 0;
	for (j = 0; j < rules->nprops; j++) {
	  sum += vals[j] * rules->weights[j];
	  weight_sum += rules->weights[j];
	}
	double_val = sum / weight_sum;
        break;
      }
      case MINIMUM_RULE: {
	double_val = vals[0];
	for (j = 1; j < rules->nprops; j++)
	  if (double_val > vals[j])
	    double_val = vals[j];
        break;
      }
      case MAXIMUM_RULE: {
	double_val = vals[0];
	for (j = 1; j < rules->nprops; j++)
	  if (double_val < vals[j])
	    double_val = vals[j];
        break;
      }
      case RANDOM_RULE: {
	double_val = vals[random_pick];
        break;
      }
      case SAME_RULE: {
	double_val = vals[0];
	for (j = 1; j < rules->nprops; j++)
	  if (double_val != vals[j]) {
	    fprintf (stderr,
    "get_new_props_ply: Error combining properties that should be the same.\n");
            exit (-1);
	  }
        break;
      }
      default:
        fprintf (stderr, "get_new_props_ply: Bad rule = %d\n",
	         rules->rule_list[i]);
	exit (-1);
    }

    /* store the combined value */

    int_val = (int) double_val;
    uint_val = (unsigned int) double_val;
    ptr = (void *) (new_data + offset);
    store_item ((char *) ptr, type, int_val, uint_val, double_val);
  }

  return ((void *) new_data);
}


/******************************************************************************
Set the list of user-specified property combination rules.
******************************************************************************/

void set_prop_rules_ply (PlyFile *ply, PlyRuleList *prop_rules)
{
  ply->rule_list = prop_rules;
}


/******************************************************************************
Append a property rule to a growing list of user-specified rules.

Entry:
  rule_list - current rule list
  name      - name of property combination rule
  property  - "element.property" says which property the rule affects

Exit:
  returns pointer to the new rule list
******************************************************************************/

PlyRuleList *append_prop_rule (
  PlyRuleList *rule_list,
  char *name,
  char *property
)
{
  PlyRuleList *rule;
  PlyRuleList *rule_ptr;
  char *str,*str2;
  char *ptr;

  /* find . */
  str = strdup (property);
  for (ptr = str; *ptr != '\0' && *ptr != '.'; ptr++) ;

  /* split string at . */
  if (*ptr == '.') {
    *ptr = '\0';
    str2 = ptr + 1;
  }
  else {
    fprintf (stderr, "Can't find property '%s' for rule '%s'\n",
             property, name);
    return (rule_list);
  }

  rule = (PlyRuleList *) malloc (sizeof (PlyRuleList));
  rule->name = name;
  rule->element = str;
  rule->property = str2;
  rule->next = NULL;

  /* either start rule list or append to it */

  if (rule_list == NULL)
    rule_list = rule;
  else {                      /* append new rule to current list */
    rule_ptr = rule_list;
    while (rule_ptr->next != NULL)
      rule_ptr = rule_ptr->next;
    rule_ptr->next = rule;
  }

  /* return pointer to list */

  return (rule_list);
}


/******************************************************************************
See if a name matches the name of any property combination rules.

Entry:
  name - name of rule we're trying to match

Exit:
  returns 1 if we find a match, 0 if not
******************************************************************************/

int matches_rule_name (char *name)
{
  int i;

  for (i = 0; rule_name_list[i].code != -1; i++)
    if (equal_strings (rule_name_list[i].name, name))
      return (1);

  return (0);
}

