/*
 * Copyright (c) 2006-2024 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author      Notes
 * 2018/08/29     Bernard     first version
 */

#include <rthw.h>

#include "dlfcn.h"
#include "dlmodule.h"
#include "dlelf.h"

#ifdef RT_USING_POSIX_FS
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#endif

#define DBG_TAG    "DLMD"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>          /* must after of DEBUG_ENABLE or some other options*/

static struct rt_module_symtab *_rt_module_symtab_begin = RT_NULL;
static struct rt_module_symtab *_rt_module_symtab_end   = RT_NULL;

#if defined(__IAR_SYSTEMS_ICC__) /* for IAR compiler */
    #pragma section="RTMSymTab"
#endif

/* set the name of module */
static void _dlmodule_set_name(struct rt_dlmodule *module, const char *path)
{
    int size;
    struct rt_object *object;
    const char *first, *end, *ptr;

    object = &(module->parent);
    ptr   = first = (char *)path;
    end   = path + rt_strlen(path);

    while (*ptr != '\0')
    {
        if (*ptr == '/')
            first = ptr + 1;
        if (*ptr == '.')
            end = ptr - 1;

        ptr ++;
    }

    size = end - first + 1;
    if (size >= RT_NAME_MAX) size = RT_NAME_MAX - 1;

    rt_strncpy(object->name, first, size);
    object->name[size] = '\0';
}

#define RT_MODULE_ARG_MAX    8
static int _rt_module_split_arg(char *cmd, rt_size_t length, char *argv[])
{
    int argc = 0;
    char *ptr = cmd;

    while ((ptr - cmd) < length)
    {
        /* strip bank and tab */
        while ((*ptr == ' ' || *ptr == '\t') && (ptr - cmd) < length)
            *ptr++ = '\0';
        /* check whether it's the end of line */
        if ((ptr - cmd) >= length) break;

        /* handle string with quote */
        if (*ptr == '"')
        {
            argv[argc++] = ++ptr;

            /* skip this string */
            while (*ptr != '"' && (ptr - cmd) < length)
                if (*ptr ++ == '\\')  ptr ++;
            if ((ptr - cmd) >= length) break;

            /* skip '"' */
            *ptr ++ = '\0';
        }
        else
        {
            argv[argc++] = ptr;
            while ((*ptr != ' ' && *ptr != '\t') && (ptr - cmd) < length)
                ptr ++;
        }

        if (argc >= RT_MODULE_ARG_MAX) break;
    }

    return argc;
}

/* invoked by main thread for exit */
static void _dlmodule_exit(void)
{
    struct rt_dlmodule *module;

    module = dlmodule_self();
    if (!module) return; /* not a module thread */

    rt_enter_critical();
    if (module->stat == RT_DLMODULE_STAT_RUNNING)
    {
        struct rt_object    *object = RT_NULL;
        struct rt_list_node *node = RT_NULL;

        /* set stat to closing */
        module->stat = RT_DLMODULE_STAT_CLOSING;

        /* suspend all threads in this module */
        for (node = module->object_list.next; node != &(module->object_list); node = node->next)
        {
            object = rt_list_entry(node, struct rt_object, list);

            if ((object->type & ~RT_Object_Class_Static) == RT_Object_Class_Thread)
            {
                rt_thread_t thread = (rt_thread_t)object;

                /* stop timer and suspend thread*/
                if ((RT_SCHED_CTX(thread).stat & RT_THREAD_STAT_MASK) != RT_THREAD_CLOSE &&
                    (RT_SCHED_CTX(thread).stat & RT_THREAD_STAT_MASK) != RT_THREAD_INIT)
                {
                    rt_timer_stop(&(thread->thread_timer));
                    rt_thread_suspend(thread);
                }
            }
        }
    }
    rt_exit_critical();

    return;
}

static void _dlmodule_thread_entry(void* parameter)
{
    int argc = 0;
    char *argv[RT_MODULE_ARG_MAX];

    struct rt_dlmodule *module = (struct rt_dlmodule*)parameter;

    if (module == RT_NULL || module->cmd_line == RT_NULL)
        /* malloc for module_cmd_line failed. */
        return;

    if (module->cmd_line)
    {
        rt_memset(argv, 0x00, sizeof(argv));
        argc = _rt_module_split_arg((char *)module->cmd_line, rt_strlen(module->cmd_line), argv);
        if (argc == 0) goto __exit;
    }

    /* set status of module */
    module->stat = RT_DLMODULE_STAT_RUNNING;

    LOG_D("run main entry: 0x%p with %s",
        module->entry_addr,
        module->cmd_line);

    if (module->entry_addr)
        module->entry_addr(argc, argv);

__exit:
    _dlmodule_exit();

    return ;
}

/**
 * @brief create a dynamic module object and initialize it.
 *
 * @return struct rt_dlmodule* If module create successfully, return the pointer to its rt_dlmodule structure.
 */
struct rt_dlmodule *dlmodule_create(void)
{
    struct rt_dlmodule *module = RT_NULL;

    module = (struct rt_dlmodule*) rt_object_allocate(RT_Object_Class_Module, "module");
    if (module)
    {
        module->stat = RT_DLMODULE_STAT_INIT;

        /* set initial priority and stack size */
        module->priority = RT_THREAD_PRIORITY_MAX - 1;
        module->stack_size = 2048;

        rt_list_init(&(module->object_list));
    }

    return module;
}

void dlmodule_destroy_subthread(struct rt_dlmodule *module, rt_thread_t thread)
{
    RT_ASSERT(thread->parent.module_id== module);

    /* lock scheduler to prevent scheduling in cleanup function. */
    rt_enter_critical();

    rt_thread_close(thread);

    /* remove thread from thread_list (defunct thread list) */
    rt_list_remove(&RT_THREAD_LIST_NODE(thread));

    /* invoke thread cleanup */
    if (thread->cleanup != RT_NULL)
        thread->cleanup(thread);

    rt_exit_critical();

#ifdef RT_USING_SIGNALS
    rt_thread_free_sig(thread);
#endif

    if (thread->parent.type & RT_Object_Class_Static)
    {
        /* detach object */
        rt_object_detach((rt_object_t)thread);
    }
#ifdef RT_USING_HEAP
    else
    {
        /* release thread's stack */
        RT_KERNEL_FREE(thread->stack_addr);
        /* delete thread object */
        rt_object_delete((rt_object_t)thread);
    }
#endif
}

/**
 * @brief destroy dynamic module and cleanup all kernel objects inside it.
 *
 * @param module Pointer to the module to be destroyed.
 * @return rt_err_t  On success, it returns RT_EOK. Otherwise, it returns the error code.
 */
rt_err_t dlmodule_destroy(struct rt_dlmodule* module)
{
    int i;

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* check parameter */
    if (module == RT_NULL)
        return -RT_ERROR;

    /* can not destroy a running module */
    if (module->stat == RT_DLMODULE_STAT_RUNNING)
        return -RT_EBUSY;

    /* do module cleanup */
    if (module->cleanup_func)
    {
        rt_enter_critical();
        module->cleanup_func(module);
        rt_exit_critical();
    }

    /* list_object(&(module->object_list));*/

    /* cleanup for all kernel objects inside module*/
    {
        struct rt_object *object = RT_NULL;
        struct rt_list_node *node = RT_NULL;

        /* detach/delete all threads in this module */
        for (node = module->object_list.next; node != &(module->object_list); )
        {
            int object_type;

            object = rt_list_entry(node, struct rt_object, list);
            object_type = object->type & ~RT_Object_Class_Static;

            /* to next node */
            node = node->next;

            if (object->type & RT_Object_Class_Static)
            {
                switch (object_type)
                {
                case RT_Object_Class_Thread:
                    dlmodule_destroy_subthread(module, (rt_thread_t)object);
                    break;
#ifdef RT_USING_SEMAPHORE
                case RT_Object_Class_Semaphore:
                    rt_sem_detach((rt_sem_t)object);
                    break;
#endif
#ifdef RT_USING_MUTEX
                case RT_Object_Class_Mutex:
                    rt_mutex_detach((rt_mutex_t)object);
                    break;
#endif
#ifdef RT_USING_EVENT
                case RT_Object_Class_Event:
                    rt_event_detach((rt_event_t)object);
                    break;
#endif
#ifdef RT_USING_MAILBOX
                case RT_Object_Class_MailBox:
                    rt_mb_detach((rt_mailbox_t)object);
                    break;
#endif
#ifdef RT_USING_MESSAGEQUEUE
                case RT_Object_Class_MessageQueue:
                    rt_mq_detach((rt_mq_t)object);
                    break;
#endif
#ifdef RT_USING_MEMHEAP
                case RT_Object_Class_MemHeap:
                    rt_memheap_detach((struct rt_memheap*)object);
                    break;
#endif
#ifdef RT_USING_MEMPOOL
                case RT_Object_Class_MemPool:
                    rt_mp_detach((struct rt_mempool*)object);
                    break;
#endif
                case RT_Object_Class_Timer:
                    rt_timer_detach((rt_timer_t)object);
                    break;
                default:
                    LOG_E("Unsupported oject type in module.");
                    break;
                }
            }
            else
            {
                switch (object_type)
                {
                case RT_Object_Class_Thread:
                    dlmodule_destroy_subthread(module, (rt_thread_t)object);
                    break;
#ifdef RT_USING_SEMAPHORE
                case RT_Object_Class_Semaphore:
                    rt_sem_delete((rt_sem_t)object);
                    break;
#endif
#ifdef RT_USING_MUTEX
                case RT_Object_Class_Mutex:
                    rt_mutex_delete((rt_mutex_t)object);
                    break;
#endif
#ifdef RT_USING_EVENT
                case RT_Object_Class_Event:
                    rt_event_delete((rt_event_t)object);
                    break;
#endif
#ifdef RT_USING_MAILBOX
                case RT_Object_Class_MailBox:
                    rt_mb_delete((rt_mailbox_t)object);
                    break;
#endif
#ifdef RT_USING_MESSAGEQUEUE
                case RT_Object_Class_MessageQueue:
                    rt_mq_delete((rt_mq_t)object);
                    break;
#endif
#ifdef RT_USING_MEMHEAP
                /* no delete operation */
#endif
#ifdef RT_USING_MEMPOOL
                case RT_Object_Class_MemPool:
                    rt_mp_delete((struct rt_mempool*)object);
                    break;
#endif
                case RT_Object_Class_Timer:
                    rt_timer_delete((rt_timer_t)object);
                    break;
                default:
                    LOG_E("Unsupported oject type in module.");
                    break;
                }
            }
        }
    }

    if (module->cmd_line) rt_free(module->cmd_line);
    /* release module symbol table */
    for (i = 0; i < module->nsym; i ++)
    {
        rt_free((void *)module->symtab[i].name);
    }
    if (module->symtab != RT_NULL)
    {
        rt_free(module->symtab);
    }

    /* destory module */
    rt_free(module->mem_space);
    /* delete module object */
    rt_object_delete((rt_object_t)module);

    return RT_EOK;
}

/**
 * @brief retrieve the dynamically loaded module that the current thread belongs to.
 *
 * @return struct rt_dlmodule* On success, it returns a pointer to the module. otherwise, it returns RT_NULL.
 */
struct rt_dlmodule *dlmodule_self(void)
{
    rt_thread_t tid;
    struct rt_dlmodule *ret = RT_NULL;

    tid = rt_thread_self();
    if (tid)
    {
        ret = (struct rt_dlmodule*) tid->parent.module_id;
    }

    return ret;
}

/*
 * Compatible with old API
 */
struct rt_dlmodule *rt_module_self(void)
{
    return dlmodule_self();
}

/**
 * @brief load an ELF module to memory.
 *
 * @param filename the path to the module to load.
 * @return struct rt_dlmodule* On success, it returns a pointer to the module object. otherwise, RT_NULL is returned.
 *
 * @note the function is used to load an ELF (Executable and Linkable Format) module from a file, validate it,
 *       and initialize it as a dynamically loaded module. what it implements are as follows:
 *       1. Load and Validate ELF: It loads an ELF file, checks its validity, and identifies it as either a relocatable or shared object.
 *       2. Memory Allocation and Cleanup: Uses rt_malloc and rt_free to allocate and free memory for module data.
 *          Error handling ensures all resources are released if an error occurs.
 *       3. Symbol Resolution and Initialization: Sets up init function and cleanup function, and calls the module_init function if it is present.
 *       4. Cache Management: Optionally (when RT_USING_CACHE defined) flushes data and invalidates instruction caches to ensure the module is correctly loaded into memory.
 */
struct rt_dlmodule* dlmodule_load(const char* filename)
{
#ifdef RT_USING_POSIX_FS
    int fd = -1, length = 0;
#endif
    rt_err_t ret = RT_EOK;
    rt_uint8_t *module_ptr = RT_NULL;
    struct rt_dlmodule *module = RT_NULL;

#ifdef RT_USING_POSIX_FS
    fd = open(filename, O_RDONLY, 0);
    if (fd >= 0)
    {
        length = lseek(fd, 0, SEEK_END);
        lseek(fd, 0, SEEK_SET);

        if (length == 0) goto __exit;

        module_ptr = (uint8_t*) rt_malloc (length);
        if (!module_ptr) goto __exit;

        if (read(fd, module_ptr, length) != length)
            goto __exit;

        /* close file and release fd */
        close(fd);
        fd = -1;
    }
    else
    {
        goto __exit;
    }
#endif

    if (!module_ptr) goto __exit;

    /* check ELF header */
    if (rt_memcmp(elf_module->e_ident, RTMMAG, SELFMAG) != 0 &&
        rt_memcmp(elf_module->e_ident, ELFMAG, SELFMAG) != 0)
    {
        rt_kprintf("Module: magic error\n");
        goto __exit;
    }

    /* check ELF class */
    if ((elf_module->e_ident[EI_CLASS] != ELFCLASS32)&&(elf_module->e_ident[EI_CLASS] != ELFCLASS64))
    {
        rt_kprintf("Module: ELF class error\n");
        goto __exit;
    }

    module = dlmodule_create();
    if (!module) goto __exit;

    /* set the name of module */
    _dlmodule_set_name(module, filename);

    LOG_D("rt_module_load: %.*s", RT_NAME_MAX, module->parent.name);

    if (elf_module->e_type == ET_REL)
    {
        ret = dlmodule_load_relocated_object(module, module_ptr);
    }
    else if (elf_module->e_type == ET_DYN)
    {
        ret = dlmodule_load_shared_object(module, module_ptr);
    }
    else
    {
        rt_kprintf("Module: unsupported elf type\n");
        goto __exit;
    }

    /* check return value */
    if (ret != RT_EOK) goto __exit;

    /* release module data */
    rt_free(module_ptr);

    /* increase module reference count */
    module->nref ++;

    /* deal with cache */
#ifdef RT_USING_CACHE
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, module->mem_space, module->mem_size);
    rt_hw_cpu_icache_ops(RT_HW_CACHE_INVALIDATE, module->mem_space, module->mem_size);
#endif

    /* set module initialization and cleanup function */
    module->init_func = dlsym(module, "module_init");
    module->cleanup_func = dlsym(module, "module_cleanup");
    module->stat = RT_DLMODULE_STAT_INIT;
    /* do module initialization */
    if (module->init_func)
    {
        module->init_func(module);
    }

    return module;

__exit:
#ifdef RT_USING_POSIX_FS
    if (fd >= 0) close(fd);
#endif
    if (module_ptr) rt_free(module_ptr);
    if (module) dlmodule_destroy(module);

    return RT_NULL;
}

/**
 * @brief load a dynamic module, and create a thread to excute the module main function.
 *
 * @param pgname path of the module to be loaded.
 * @param cmd the command string (with commandline options) for startup module.
 * @param cmd_size the command's length.
 * @return struct rt_dlmodule* On success, it returns a pointer to the module object. otherwise, RT_NULL is returned.
 */
struct rt_dlmodule* dlmodule_exec(const char* pgname, const char* cmd, int cmd_size)
{
    struct rt_dlmodule *module = RT_NULL;

    module = dlmodule_load(pgname);
    if (module)
    {
        if (module->entry_addr)
        {
            /* exec this module */
            rt_thread_t tid;

            module->cmd_line = rt_strdup(cmd);

            /* check stack size and priority */
            if (module->priority > RT_THREAD_PRIORITY_MAX) module->priority = RT_THREAD_PRIORITY_MAX - 1;
            if (module->stack_size < 2048 || module->stack_size > (1024 * 32)) module->stack_size = 2048;

            tid = rt_thread_create(module->parent.name, _dlmodule_thread_entry, (void*)module,
                module->stack_size, module->priority, 10);
            if (tid)
            {
                tid->parent.module_id= module;
                module->main_thread = tid;

                rt_thread_startup(tid);
            }
            else
            {
                /* destory dl module */
                dlmodule_destroy(module);
                module = RT_NULL;
            }
        }
    }

    return module;
}

#if defined(RT_USING_CUSTOM_DLMODULE)
struct rt_dlmodule* dlmodule_load_custom(const char* filename, struct rt_dlmodule_ops* ops)
{
#ifdef RT_USING_POSIX_FS
    int fd = -1, length = 0;
#endif
    rt_err_t ret = RT_EOK;
    rt_uint8_t *module_ptr = RT_NULL;
    struct rt_dlmodule *module = RT_NULL;

    if (ops)
    {
        RT_ASSERT(ops->load);
        RT_ASSERT(ops->unload);
        module_ptr = ops->load(filename);
    }
#ifdef RT_USING_POSIX_FS
    else
    {
        fd = open(filename, O_RDONLY, 0);
        if (fd >= 0)
        {
            length = lseek(fd, 0, SEEK_END);
            lseek(fd, 0, SEEK_SET);

            if (length == 0) goto __exit;

            module_ptr = (uint8_t*) rt_malloc (length);
            if (!module_ptr) goto __exit;

            if (read(fd, module_ptr, length) != length)
                goto __exit;

            /* close file and release fd */
            close(fd);
            fd = -1;
        }
        else
        {
            goto __exit;
        }
    }
#endif

    if (!module_ptr) goto __exit;

    /* check ELF header */
    if (rt_memcmp(elf_module->e_ident, RTMMAG, SELFMAG) != 0 &&
        rt_memcmp(elf_module->e_ident, ELFMAG, SELFMAG) != 0)
    {
        rt_kprintf("Module: magic error\n");
        goto __exit;
    }

    /* check ELF class */
    if (elf_module->e_ident[EI_CLASS] != ELFCLASS32)
    {
        rt_kprintf("Module: ELF class error\n");
        goto __exit;
    }

    module = dlmodule_create();
    if (!module) goto __exit;

    /* set the name of module */
    _dlmodule_set_name(module, filename);

    LOG_D("rt_module_load: %.*s", RT_NAME_MAX, module->parent.name);

    if (elf_module->e_type == ET_REL)
    {
        ret = dlmodule_load_relocated_object(module, module_ptr);
    }
    else if (elf_module->e_type == ET_DYN)
    {
        ret = dlmodule_load_shared_object(module, module_ptr);
    }
    else
    {
        rt_kprintf("Module: unsupported elf type\n");
        goto __exit;
    }

    /* check return value */
    if (ret != RT_EOK) goto __exit;

    /* release module data */
    if (ops)
    {
        ops->unload(module_ptr);
    }
    else
    {
        rt_free(module_ptr);
    }

    /* increase module reference count */
    module->nref ++;

    /* deal with cache */
#ifdef RT_USING_CACHE
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, module->mem_space, module->mem_size);
    rt_hw_cpu_icache_ops(RT_HW_CACHE_INVALIDATE, module->mem_space, module->mem_size);
#endif

    /* set module initialization and cleanup function */
    module->init_func = dlsym(module, "module_init");
    module->cleanup_func = dlsym(module, "module_cleanup");
    module->stat = RT_DLMODULE_STAT_INIT;
    /* do module initialization */
    if (module->init_func)
    {
        module->init_func(module);
    }

    return module;

__exit:
#ifdef RT_USING_POSIX_FS
    if (fd >= 0) close(fd);
#endif
    if (module_ptr)
    {
        if (ops)
        {
            ops->unload(module_ptr);
        }
        else
        {
            rt_free(module_ptr);
        }
    }

    if (module) dlmodule_destroy(module);

    return RT_NULL;
}

struct rt_dlmodule* dlmodule_exec_custom(const char* pgname, const char* cmd, int cmd_size, struct rt_dlmodule_ops* ops)
{
    struct rt_dlmodule *module = RT_NULL;

    module = dlmodule_load_custom(pgname, ops);
    if (module)
    {
        if (module->entry_addr)
        {
            /* exec this module */
            rt_thread_t tid;

            module->cmd_line = rt_strdup(cmd);

            /* check stack size and priority */
            if (module->priority > RT_THREAD_PRIORITY_MAX) module->priority = RT_THREAD_PRIORITY_MAX - 1;
            if (module->stack_size < 2048 || module->stack_size > (1024 * 32)) module->stack_size = 2048;

            tid = rt_thread_create(module->parent.name, _dlmodule_thread_entry, (void*)module,
                module->stack_size, module->priority, 10);
            if (tid)
            {
                tid->parent.module_id= module;
                module->main_thread = tid;

                rt_thread_startup(tid);
            }
            else
            {
                /* destory dl module */
                dlmodule_destroy(module);
                module = RT_NULL;
            }
        }
    }

    return module;
}
#endif

/**
 * @brief exit a dynamically loaded module.
 *
 * @param ret_code the return code for module exit.
 *
 * @note this function is responsible for gracefully exiting a dynamically loaded module, releasing resources associated with the module,
 *       and handling cleanup operations. what it implements are as follows:
 *       1. Thread and Resource Cleanup: The function safely exits a module by deleting its main thread and freeing resources associated with it.
 *       2. Status Management: Checks and updates the module's state, setting a return code and calling _dlmodule_exit() to transition to a closing state.
 *       3. Critical Sections: Critical sections ensure that the exit process is atomic and free from race conditions.
 */
void dlmodule_exit(int ret_code)
{
    rt_thread_t thread;
    struct rt_dlmodule *module;

    module = dlmodule_self();
    if (!module) return;

    /* disable scheduling */
    rt_enter_critical();

    /* module is not running */
    if (module->stat != RT_DLMODULE_STAT_RUNNING)
    {
        /* restore scheduling */
        rt_exit_critical();

        return;
    }

    /* set return code */
    module->ret_code = ret_code;

    /* do exit for this module */
    _dlmodule_exit();
    /* the stat of module was changed to CLOSING in _dlmodule_exit */

    thread = module->main_thread;
    if ((RT_SCHED_CTX(thread).stat & RT_THREAD_STAT_MASK) == RT_THREAD_CLOSE)
    {
        /* main thread already closed */
        rt_exit_critical();

        return ;
    }

    /* delete thread: insert to defunct thread list */
    rt_thread_delete(thread);
    /* enable scheduling */
    rt_exit_critical();
}

/**
 * @brief search for a symbol by its name in the kernel symbol table.
 *
 * @param sym_str the symbol name string.
 * @return rt_uint32_t On success, it returns the address of the symbol.
 *         Otherwise, it returns 0 (indicating the symbol was not found).
 */
rt_uint32_t dlmodule_symbol_find(const char *sym_str)
{
    /* find in kernel symbol table */
    struct rt_module_symtab *index;

    for (index = _rt_module_symtab_begin; index != _rt_module_symtab_end; index ++)
    {
        if (rt_strcmp(index->name, sym_str) == 0)
            return (rt_uint32_t)index->addr;
    }

    return 0;
}

int rt_system_dlmodule_init(void)
{
#if defined(__GNUC__) && !defined(__CC_ARM)
    extern int __rtmsymtab_start;
    extern int __rtmsymtab_end;

    _rt_module_symtab_begin = (struct rt_module_symtab *)&__rtmsymtab_start;
    _rt_module_symtab_end   = (struct rt_module_symtab *)&__rtmsymtab_end;
#elif defined (__CC_ARM)
    extern int RTMSymTab$$Base;
    extern int RTMSymTab$$Limit;

    _rt_module_symtab_begin = (struct rt_module_symtab *)&RTMSymTab$$Base;
    _rt_module_symtab_end   = (struct rt_module_symtab *)&RTMSymTab$$Limit;
#elif defined (__IAR_SYSTEMS_ICC__)
    _rt_module_symtab_begin = __section_begin("RTMSymTab");
    _rt_module_symtab_end   = __section_end("RTMSymTab");
#endif

    return 0;
}
INIT_COMPONENT_EXPORT(rt_system_dlmodule_init);

/**
 * This function will find the specified module.
 *
 * @param name the name of module finding
 *
 * @return the module
 */
struct rt_dlmodule *dlmodule_find(const char *name)
{
    rt_object_t object;
    struct rt_dlmodule *ret = RT_NULL;

    object = rt_object_find(name, RT_Object_Class_Module);
    if (object)
    {
        ret = (struct rt_dlmodule*) object;
    }

    return ret;
}
RTM_EXPORT(dlmodule_find);

int list_symbols(void)
{
    extern int __rtmsymtab_start;
    extern int __rtmsymtab_end;

    /* find in kernel symbol table */
    struct rt_module_symtab *index;

    for (index = _rt_module_symtab_begin;
         index != _rt_module_symtab_end;
         index ++)
    {
        rt_kprintf("%s => 0x%08x\n", index->name, index->addr);
    }

    return 0;
}
MSH_CMD_EXPORT(list_symbols, list symbols information);

int list_module(void)
{
    struct rt_dlmodule *module;
    struct rt_list_node *list, *node;
    struct rt_object_information *info;

    info = rt_object_get_information(RT_Object_Class_Module);
    list = &info->object_list;

    rt_kprintf("module   ref      address \n");
    rt_kprintf("-------- -------- ------------\n");
    for (node = list->next; node != list; node = node->next)
    {
        module = (struct rt_dlmodule *)(rt_list_entry(node, struct rt_object, list));
        rt_kprintf("%-*.*s %-04d  0x%08x\n",
                   RT_NAME_MAX, RT_NAME_MAX, module->parent.name, module->nref, module->mem_space);
    }

    return 0;
}
MSH_CMD_EXPORT(list_module, list modules in system);
