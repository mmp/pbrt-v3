/*

Header for PLY polygon files.

- Greg Turk

A PLY file contains a single polygonal _object_.

An object is composed of lists of _elements_.  Typical elements are
vertices, faces, edges and materials.

Each type of element for a given object has one or more _properties_
associated with the element type.  For instance, a vertex element may
have as properties three floating-point values x,y,z and three unsigned
chars for red, green and blue.

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

#ifndef __PLY_H__
#define __PLY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stddef.h>

#define PLY_ASCII      1        /* ascii PLY file */
#define PLY_BINARY_BE  2        /* binary PLY file, big endian */
#define PLY_BINARY_LE  3        /* binary PLY file, little endian */

#define PLY_OKAY    0           /* ply routine worked okay */
#define PLY_ERROR  -1           /* error in ply routine */

/* scalar data types supported by PLY format */

#define StartType  0
#define Int8       1
#define Int16      2
#define Int32      3
#define Uint8      4
#define Uint16     5
#define Uint32     6
#define Float32    7
#define Float64    8
#define EndType    9

#define  PLY_SCALAR  0
#define  PLY_LIST    1
#define  PLY_STRING  2


typedef struct PlyProperty {    /* description of a property */

  char *name;                   /* property name */
  int external_type;            /* file's data type */
  int internal_type;            /* program's data type */
  int offset;                   /* offset bytes of prop in a struct */

  int is_list;                  /* 0 = scalar, 1 = list, 2 = char string */
  int count_external;           /* file's count type */
  int count_internal;           /* program's count type */
  int count_offset;             /* offset byte for list count */

} PlyProperty;

typedef struct PlyElement {     /* description of an element */
  char *name;                   /* element name */
  int num;                      /* number of elements in this object */
  int size;                     /* size of element (bytes) or -1 if variable */
  int nprops;                   /* number of properties for this element */
  PlyProperty **props;          /* list of properties in the file */
  char *store_prop;             /* flags: property wanted by user? */
  int other_offset;             /* offset to un-asked-for props, or -1 if none*/
  int other_size;               /* size of other_props structure */
} PlyElement;

typedef struct PlyOtherProp {   /* describes other properties in an element */
  char *name;                   /* element name */
  int size;                     /* size of other_props */
  int nprops;                   /* number of properties in other_props */
  PlyProperty **props;          /* list of properties in other_props */
} PlyOtherProp;

typedef struct OtherData { /* for storing other_props for an other element */
  void *other_props;
} OtherData;

typedef struct OtherElem {     /* data for one "other" element */
  char *elem_name;             /* names of other elements */
  int elem_count;              /* count of instances of each element */
  OtherData **other_data;      /* actual property data for the elements */
  PlyOtherProp *other_props;   /* description of the property data */
} OtherElem;

typedef struct PlyOtherElems {  /* "other" elements, not interpreted by user */
  int num_elems;                /* number of other elements */
  OtherElem *other_list;        /* list of data for other elements */
} PlyOtherElems;

#define AVERAGE_RULE  1
#define MAJORITY_RULE 2
#define MINIMUM_RULE  3
#define MAXIMUM_RULE  4
#define SAME_RULE     5
#define RANDOM_RULE   6

typedef struct PlyPropRules {   /* rules for combining "other" properties */
  PlyElement *elem;      /* element whose rules we are making */
  int *rule_list;        /* types of rules (AVERAGE_PLY, MAJORITY_PLY, etc.) */
  int nprops;            /* number of properties we're combining so far */
  int max_props;         /* maximum number of properties we have room for now */
  void **props;          /* list of properties we're combining */
  float *weights;        /* list of weights of the properties */
} PlyPropRules;

typedef struct PlyRuleList {
  char *name;                  /* name of the rule */
  char *element;               /* name of element that rule applies to */
  char *property;              /* name of property that rule applies to */
  struct PlyRuleList *next;    /* pointer for linked list of rules */
} PlyRuleList;

typedef struct PlyFile {        /* description of PLY file */
  FILE *fp;                     /* file pointer */
  int file_type;                /* ascii or binary */
  float version;                /* version number of file */
  int num_elem_types;           /* number of element types of object */
  PlyElement **elems;           /* list of elements */
  int num_comments;             /* number of comments */
  char **comments;              /* list of comments */
  int num_obj_info;             /* number of items of object information */
  char **obj_info;              /* list of object info items */
  PlyElement *which_elem;       /* element we're currently reading or writing */
  PlyOtherElems *other_elems;   /* "other" elements from a PLY file */
  PlyPropRules *current_rules;  /* current propagation rules */
  PlyRuleList *rule_list;       /* rule list from user */
} PlyFile;

/* memory allocation */
/*
extern char *my_alloc();
*/
#define myalloc(mem_size) my_alloc((mem_size), __LINE__, __FILE__)


/* old routines */

#if 0
extern PlyFile *ply_write(FILE *, int, char **, int);
extern PlyFile *ply_read(FILE *, int *, char ***);
extern PlyFile *ply_open_for_reading( char *, int *, char ***, int *, float *);
extern void ply_close(PlyFile *);
extern PlyOtherProp *ply_get_other_properties(PlyFile *, char *, int);
#endif

extern void ply_describe_property(PlyFile *, char *, PlyProperty *);
extern void ply_get_property(PlyFile *, char *, PlyProperty *);
extern void ply_get_element(PlyFile *, void *);


/*** delcaration of routines ***/

PlyOtherElems *get_other_element_ply (PlyFile *);

PlyFile *read_ply(FILE *);
PlyFile *write_ply(FILE *, int, char **, int);
extern PlyFile *open_for_writing_ply(char *, int, char **, int);
void close_ply(PlyFile *);
void free_ply(PlyFile *);

void get_info_ply(PlyFile *, float *, int *);
void free_other_elements_ply (PlyOtherElems *);

void append_comment_ply(PlyFile *, char *);
void append_obj_info_ply(PlyFile *, char *);
void copy_comments_ply(PlyFile *, PlyFile *);
void copy_obj_info_ply(PlyFile *, PlyFile *);
char **get_comments_ply(PlyFile *, int *);
char **get_obj_info_ply(PlyFile *, int *);

char **get_element_list_ply(PlyFile *, int *);
void setup_property_ply(PlyFile *, PlyProperty *);
void get_element_ply (PlyFile *, void *);
char *setup_element_read_ply (PlyFile *, int, int *);
PlyOtherProp *get_other_properties_ply(PlyFile *, int);

void element_count_ply(PlyFile *, char *, int);
void describe_element_ply(PlyFile *, char *, int);
void describe_property_ply(PlyFile *, PlyProperty *);
void describe_other_properties_ply(PlyFile *, PlyOtherProp *, int);
void describe_other_elements_ply ( PlyFile *, PlyOtherElems *);
void get_element_setup_ply(PlyFile *, char *, int, PlyProperty *);
PlyProperty **get_element_description_ply(PlyFile *, char *, int*, int*);
void element_layout_ply(PlyFile *, char *, int, int, PlyProperty *);

void header_complete_ply(PlyFile *);
void put_element_setup_ply(PlyFile *, char *);
void put_element_ply(PlyFile *, void *);
void put_other_elements_ply(PlyFile *);

PlyPropRules *init_rule_ply (PlyFile *, char *);
void modify_rule_ply (PlyPropRules *, char *, int);
void start_props_ply (PlyFile *, PlyPropRules *);
void weight_props_ply (PlyFile *, float, void *);
void *get_new_props_ply(PlyFile *);
void set_prop_rules_ply (PlyFile *, PlyRuleList *);
PlyRuleList *append_prop_rule (PlyRuleList *, char *, char *);
int matches_rule_name (char *);

int equal_strings(char *, char *);
char *recreate_command_line (int, char *argv[]);


#ifdef __cplusplus
}
#endif
#endif /* !__PLY_H__ */

