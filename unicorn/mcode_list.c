#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
#include "list.h"

#define debug 0 

static struct M_list  blist;

void init_MCode_list() 
{
  INIT_LIST_HEAD(&blist.list);
}

void destroy_MCode_list() 
{
  struct M_list* iter; 
  /* remove all items in the list */
  while( !list_empty(&blist.list) ) {
    iter = list_entry(blist.list.next,struct M_list,list);
    if(debug)
        printf("remove %d %s \n", iter->no, iter->MCode);
    list_del(&iter->list);
    free(iter);
  }
}

struct M_list* get_list_item()
{
  struct M_list* iter; 
  if ( !list_empty(&blist.list) ) {
    iter = list_entry(blist.list.next,struct M_list,list);
    //if(debug)
     //   printf("get list item %d %s \n", iter->no, iter->MCode);
    return iter;
  } else 
    return NULL;
}

void del_list_item(struct M_list* item)
{
    if ( item != NULL ) {
        if(debug)
            printf("del list item %d %s \n", item->no, item->MCode);
        list_del(&item->list);
        free(item);
    }
}

void append_list(const char* mCode, int no)          
{
  struct M_list* tmp;
  tmp = (struct M_list*)malloc(sizeof(struct M_list));
  if(!tmp) {
    perror("malloc for append_list");
	return;
  }
  memset(tmp->MCode, 0, sizeof(tmp->MCode));
  strcpy(tmp->MCode, mCode);
  tmp->no = no;
  if(debug)
    printf("add M code %d %s \n", tmp->no, tmp->MCode);

  list_add_tail( &(tmp->list), &(blist.list) );
}

void debug_MCode_list() 
{
  struct M_list* iter; 
  /* iterates list */
  list_for_each_entry(iter,&blist.list,list) {
    printf("No:%d %s \n", iter->no, iter->MCode);
  }
}

#if 0
static int main_1()
{
  /* add item to list   
  append_list(&blist, "NM", 87501);
  append_list(&blist, "CA", 94041);
  append_list(&blist, "IL", 60561);
   */
  return 0;
}
#endif

