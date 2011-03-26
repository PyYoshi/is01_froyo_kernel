/* This program can be distributed under the terms of the GNU v2. See the file SHPROC_COPYING. */
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/stat.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/dirent.h>
#include <linux/unistd.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/fdtable.h>
static void shproc_release( struct kobject *kobj );
static ssize_t shproc_sysfs_show( struct kobject *kobj, struct attribute *attr, char *buf );
static ssize_t shproc_sysfs_store( struct kobject *kobj, struct attribute *attr, const char *buf, size_t count );

typedef struct {
    const char *name;
    struct kobject *kobj;
    struct kobj_type *ktype;
} shproc_data;

static struct kset *shproc_kset;
static struct kobject shproc_kobj;

static struct sysfs_ops shproc_sysfs_ops = {
    .show  = shproc_sysfs_show,
    .store = shproc_sysfs_store,
};

static struct kobj_attribute shproc_attribute =
    __ATTR( sd, 0600, NULL, NULL );

static struct attribute *shproc_attrs[] = {
    &shproc_attribute.attr,
    NULL,
};

static struct kobj_type shproc_ktype = {
    .release = shproc_release,
    .sysfs_ops = &shproc_sysfs_ops,
    .default_attrs = shproc_attrs,
};
static shproc_data data = {
    .name  = "data",
    .kobj  = &shproc_kobj,
    .ktype = &shproc_ktype,
};

static void shproc_release( struct kobject *kobj )
{
    kfree( kobj );
}

static ssize_t shproc_sysfs_show( struct kobject *kobj, struct attribute *attr,
                                  char *buf )
{
    return 0;
}

static ssize_t shproc_sysfs_store( struct kobject *kobj, struct attribute *attr,
                                   const char *buf, size_t count )
{
    struct task_struct *p;
    int pid, len;
    unsigned int i;
    int ret = -1;
    char *buff = NULL;
    char *pathname;

    if( buf == NULL ){
        return ret;
    }

    pid = simple_strtol( buf, NULL, 10 );
    if( pid <= 0 ){
        printk( "[vold]pid(%d) error\n", pid );
        return ret;
    }

    buff = kmalloc( PAGE_SIZE, GFP_KERNEL );
    if( !buff ){
        printk( "[vold]kmalloc error\n" );
        return ret;
    }

    rcu_read_lock();

    p = find_task_by_vpid( (pid_t)pid );
    if( !p ){
        rcu_read_unlock();
        kfree( buff );
        printk( "[vold]task_struct get error\n" );
        return ret;
    }

    for( i = 0; i < p->files->fdt->max_fds; i++ ){
        memset( buff, 0x00, PAGE_SIZE );
        if( p->files->fdt->fd[i] == NULL ){
            break;
        }
        pathname = d_path( &(p->files->fdt->fd[i]->f_path) ,buff, PAGE_SIZE );
        ret = PTR_ERR(pathname);
        if( IS_ERR(pathname) ){
            break;
        }
        len = strlen( pathname );
        if( len > 7 ){
            len = 7;
        }
        if( !strncmp(pathname, "/sdcard", len) ){
            printk( "[vold]find pid:%d\n", pid );
            ret = len;
            break;
        }
    }

    rcu_read_unlock();
    kfree( buff );
    return ret;
}

static int __init shproc_init( void )
{
    int ret;

    /* Create a kset with the name of "shproc" */
    /* located under /sys/kernel/ */
    shproc_kset = kset_create_and_add( "shproc", NULL, kernel_kobj );
    if( !shproc_kset ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        return -ENOMEM;
    }

    data.kobj->kset = shproc_kset;
    ret = kobject_init_and_add( data.kobj, data.ktype, NULL, "%s", data.name );
    if( ret ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        kobject_put( data.kobj );
    }

    return ret;
}

static void __exit shproc_exit( void )
{
    kset_unregister( shproc_kset );
}

module_init( shproc_init );
module_exit( shproc_exit );

