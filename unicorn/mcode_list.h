
#ifndef _MCODE_LIST_H
#define _MCODE_LIST_H

#define MCodeArrayMaxSize  50
struct M_list
{
  struct list_head list;
  int   no;
  char MCode[MCodeArrayMaxSize];
};


void init_MCode_list();
void destroy_MCode_list();
void append_list(const char* mCode, int no);
void debug_MCode_list();
struct M_list* get_list_item();
void del_list_item(struct M_list* item);

#endif
