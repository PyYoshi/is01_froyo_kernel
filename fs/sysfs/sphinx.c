/*
 * Copyright (C) 2009 Sharp.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/binfmts.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/smp_lock.h>
#include <linux/delay.h>
#include <mach/sharp_smem.h>

#include <linux/proc_fs.h>

// #define USES_UEVENT_NOTIFY
#define DIGEST_SIZE 64
#define DGSTMGRD_SIG "dgstmgrd:"
#define DGSTMGRD_SIG_SIZE (9)
#define DGSTMGRD_KEY_LENGTH (32)
#define sphinx_printk if(0)printk

typedef enum
{
	SPHINX_LOAD_PROCESS_ELF,
	SPHINX_LOAD_PROCESS_LINKER,
	SPHINX_LOAD_PROCESS_DEX,
	SPHINX_UNLOAD_PROCESS,
	SPHINX_KILL_PROCESS,
} sphinx_type;

typedef struct
{
	sphinx_type type;
	pid_t pid;
	unsigned char digest[DIGEST_SIZE];
	char name[128];
	char name2[128];
} sphinx_data;

typedef struct _sphinx_node
{
	sphinx_data data;
	struct _sphinx_node *prev;
} sphinx_node;
 
typedef struct
{
	sphinx_node *head;
	sphinx_node *tail;
} sphinx_queue_struct;

typedef struct
{
	const char *name;
	struct kobject *kobj;
	struct kobj_type *ktype;
} kobj_data;

typedef struct
{
	void* next;
	char buffer[256];
} sphinx_proven_struct;

static sphinx_proven_struct* sphinx_proven_processes = NULL;
static pid_t digest_manager_pid = -1;
static int sphinx_pw_mode = 0;
static char sphinx_hold_key[DGSTMGRD_KEY_LENGTH];

#define CREATE_KOBJECT(dir, file)										\
	static void dir##_release(struct kobject *kobj);					\
	static ssize_t dir##_show(struct kobject *kobj, struct attribute *attr, char *buf); \
	static ssize_t dir##_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len);	\
	static struct kobject dir##_kobj;									\
	static struct kobj_attribute dir##_attribute = __ATTR(file, 0666, NULL, NULL);	\
	static struct attribute *dir##_default_attrs[] = {					\
		&dir##_attribute.attr,											\
		NULL,	/* need to NULL terminate the list of attributes */		\
	};																	\
	static struct sysfs_ops dir##_sysfs_ops = {						\
		.show = dir##_show,											\
		.store = dir##_store,											\
	};																	\
	static struct kobj_type dir##_ktype = {							\
		.release = dir##_release,										\
		.sysfs_ops = &dir##_sysfs_ops,									\
		.default_attrs = dir##_default_attrs,							\
	};																	\
	static kobj_data dir##_data = {										\
		.name = #dir,													\
		.kobj = &dir##_kobj,											\
		.ktype = &dir##_ktype,											\
	};																	\

extern void presenter_set_readable(void);

int sphinx_elf_register_process(struct file *file);
int sphinx_elf_unregister_process(pid_t pid);

static int sphinx_elf_create_digest(struct file *file, unsigned char *digest);
static int sphinx_queue_put(sphinx_data *data);
static int sphinx_queue_get(sphinx_data *data);
static int sphinx_notify_uevent(sphinx_data *data);
static int sphinx_create_kobject(kobj_data *data);

static struct kset *sphinx_kset;
static sphinx_queue_struct sphinx_queue = {NULL, NULL};

CREATE_KOBJECT(elfloader, digestsender);
CREATE_KOBJECT(provenprocess, pids);
CREATE_KOBJECT(secure, key);
CREATE_KOBJECT(dexloader, pathsender);
CREATE_KOBJECT(package, name);

pid_t sphinx_get_digest_manager_pid(void)
{
	return digest_manager_pid;
}

int sphinx_elf_register_process(struct file *file)
{
	int ret;
	sphinx_data data;

	memset(&data, 0x00, sizeof(sphinx_data));

	data.type = SPHINX_LOAD_PROCESS_ELF;
	data.pid = current->pid;
	memset(data.name, 0, 128);
	strncpy(data.name, current->comm, 127);

	ret = sphinx_elf_create_digest(file, data.digest);
	if(ret) return ret;

	ret = sphinx_notify_uevent(&data);

	return ret;
}

int sphinx_elf_unregister_process(pid_t pid)
{
	int ret;
	sphinx_data data;
	sphinx_proven_struct* curr;

	memset(&data, 0x00, sizeof(sphinx_data));

	data.type = SPHINX_UNLOAD_PROCESS;
	data.pid = pid;

	ret = sphinx_notify_uevent(&data);

	if(pid == digest_manager_pid)
	{
		digest_manager_pid = 0;

		if(sphinx_proven_processes != NULL)
		{
			for(curr = sphinx_proven_processes; curr != NULL; curr = (sphinx_proven_struct*)curr->next)
			{
				if(strstr(curr->buffer, DGSTMGRD_SIG) == curr->buffer)
				{
					memset(curr->buffer, 0, 256);
					sprintf(curr->buffer, "%s0", DGSTMGRD_SIG);
				}
			}
		}
	}

	return ret;
}

static int sphinx_elf_create_digest(struct file *file, unsigned char *digest)
{
	struct crypto_hash* hash;
	struct hash_desc desc;
	unsigned char* buff;
	int n;
	int p = 0;
	
	hash = crypto_alloc_hash("sha256", 0, CRYPTO_ALG_ASYNC);
	if(hash == NULL) {
		return -1;
	}

	buff = (unsigned char*)kmalloc(1024, GFP_KERNEL);

	if(buff == NULL) {
		return -1;
	}

	desc.tfm = hash;
	desc.flags = CRYPTO_TFM_REQ_MAY_SLEEP;
	
	crypto_hash_init(&desc);

	while((n = kernel_read(file, p, buff, 1024)) > 0) {
		struct scatterlist sg;

		sg_init_one(&sg, buff, n);

		crypto_hash_update(&desc, &sg, n);

		p += n;
	}

	crypto_hash_final(&desc, digest);

	crypto_free_hash(desc.tfm);
	kfree(buff);

	return 0;
}

static int sphinx_queue_put(sphinx_data *data)
{
	sphinx_node *new_queue = NULL;

	lock_kernel();

	new_queue = (sphinx_node*)kmalloc(sizeof(sphinx_node), GFP_KERNEL);
	if(new_queue == NULL){
		unlock_kernel();
		return -ENOMEM;
	}

	memcpy(&new_queue->data, data, sizeof(sphinx_data));
	new_queue->prev = NULL;

	if(sphinx_queue.tail == NULL){
		sphinx_queue.tail = new_queue;
	}
	else{
		sphinx_queue.head->prev = new_queue;
	}
	sphinx_queue.head = new_queue;

	unlock_kernel();

	return 0;
}
 
static int sphinx_queue_get(sphinx_data *data)
{
	sphinx_node *last_queue = NULL;

	lock_kernel();

	last_queue = sphinx_queue.tail;
	if(last_queue == NULL){
		unlock_kernel();
		return -ENOENT;
	}

	sphinx_queue.tail = last_queue->prev;
	memcpy(data, last_queue, sizeof(sphinx_data));

	kfree(last_queue);

	unlock_kernel();

	return 0;
}

static int sphinx_notify_uevent(sphinx_data *data)
{
	int ret;

	ret = sphinx_queue_put(data);
	if(ret){
		sphinx_printk(KERN_ALERT "sphinx_digest queue_put: %d\n", ret);
		return ret;
	}

#ifdef USES_UEVENT_NOTIFY
	kobject_uevent(&elfloader_kobj, KOBJ_CHANGE);
#else
	presenter_set_readable();
#endif /* USES_UEVENT_NOTIFY */

	return 0;
}

static void elfloader_release(struct kobject *kobj)
{
	kfree(kobj);
}

static ssize_t elfloader_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	if(current->pid != digest_manager_pid) return 0;

	ret = sphinx_queue_get((sphinx_data*)buf);
	if(ret) return 0;

	return sizeof(sphinx_data);
}

static ssize_t elfloader_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	int ret;
	sphinx_data *data = (sphinx_data*)buf;

	data->pid = current->pid;
	memset(data->name2, 0, 128);	
	strncpy(data->name2, current->comm, 127);

	ret = sphinx_notify_uevent(data);
	if(ret) return 0;

	return len;
}

static void provenprocess_release(struct kobject *kobj)
{
	kfree(kobj);
}

static ssize_t provenprocess_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sphinx_proven_struct* curr;
	
	for(curr = sphinx_proven_processes; curr != NULL; curr = (sphinx_proven_struct*)curr->next)
	{
		sprintf(buf + ret, "%s\n", curr->buffer);
		ret += (strlen(curr->buffer) + 1);
	}

	return ret;
}

static ssize_t provenprocess_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	sphinx_proven_struct* prev;
	sphinx_proven_struct* curr;
	sphinx_proven_struct* node;
	int n;

	if(current->pid != 1)return 0;

	if(strchr(buf, ':') == NULL)return 0;

	n = (uint32_t)strchr(buf, ':') - (uint32_t)buf;

	if(n < 0)return 0;

	if(sphinx_proven_processes != NULL)
	{
		for(curr = sphinx_proven_processes, prev = sphinx_proven_processes; curr != NULL; curr = (sphinx_proven_struct*)curr->next)
		{
			if(memcmp(buf, curr->buffer, n) == 0)return 0;
			
			prev = curr;
		}

		node = (sphinx_proven_struct*)kmalloc(sizeof(sphinx_proven_struct), GFP_KERNEL);

		if(node == NULL)return 0;

		memset(node, 0, sizeof(sphinx_proven_struct));
		memcpy(node->buffer, buf, len);
		
		prev->next = (void*)node;
	}
	else
	{
		sphinx_proven_processes = (sphinx_proven_struct*)kmalloc(sizeof(sphinx_proven_struct), GFP_KERNEL);

		if(sphinx_proven_processes == NULL)return 0;

		memset(sphinx_proven_processes, 0, sizeof(sphinx_proven_struct));
		memcpy(sphinx_proven_processes->buffer, buf, len);
	}

	if((digest_manager_pid == -1) && (strstr(buf, DGSTMGRD_SIG) == buf))
	{
		const char* ptr = buf + DGSTMGRD_SIG_SIZE;
		char tmp[256];
		char* ptmp = &tmp[0];
		
		memset(tmp, 0, 256);
		memcpy(tmp, ptr, len - DGSTMGRD_SIG_SIZE);
		
		for(digest_manager_pid = 0; *ptmp != '\0' && *ptmp >= '0' && *ptmp <= '9'; ptmp++)
		{
			digest_manager_pid = 10 * digest_manager_pid + (*ptmp - '0');
		}
	}

	return len;
}

static void secure_release(struct kobject *kobj)
{
	kfree(kobj);
}

static ssize_t secure_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	sharp_smem_common_type* sh_smem_common = NULL;
	int i;
	int len = DGSTMGRD_KEY_LENGTH;
	char pw[DGSTMGRD_KEY_LENGTH];
	
	if(sphinx_pw_mode != 1)return 0;

	if(current->pid != digest_manager_pid)
	{
		sphinx_pw_mode = -1;
		return 0;
	}

	sh_smem_common = sh_smem_get_common_address();

	if(sh_smem_common == NULL)
	{
		sphinx_pw_mode = -1;
		return 0;
	}

	memcpy(pw, sh_smem_common->shsecure_PassPhrase, DGSTMGRD_KEY_LENGTH);
	memset(sh_smem_common->shsecure_PassPhrase, 0, DGSTMGRD_KEY_LENGTH);

	for(i = 0; i < len; i++)
	{
		buf[i] = pw[i] ^ sphinx_hold_key[i%32];
	}

	sphinx_pw_mode = -1;

	return len;
}

static ssize_t secure_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	if(sphinx_pw_mode != 0)return 0;
	
	if(current->pid != digest_manager_pid)
	{
		sphinx_pw_mode = -1;

		return 0;
	}

	if(len != DGSTMGRD_KEY_LENGTH)
	{
		sphinx_pw_mode = -1;

		return 0;
	}

	memcpy(sphinx_hold_key, buf, DGSTMGRD_KEY_LENGTH);

	sphinx_pw_mode = 1;

	return 32;
}

static void dexloader_release(struct kobject *kobj)
{
	kfree(kobj);
}

static ssize_t dexloader_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return 0;
}

static ssize_t dexloader_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	int ret;
	sphinx_data data;

//	if(len != 32)return 0;

	memset(&data, 0, sizeof(sphinx_data));
	
	data.type = SPHINX_LOAD_PROCESS_ELF;//DEX;
	data.pid = current->pid;
	memset(data.name, 0, 128);
	strncpy(data.name, current->comm, 127);
	memcpy(data.digest, buf, 32);

	ret = sphinx_notify_uevent(&data);
	if(ret) return 0;

	return len;
}

static void package_release(struct kobject *kobj)
{
	kfree(kobj);
}

static pid_t package_held_pid = -1;

#define PACKAGE_SHOW_MAX_LEN (256)

static ssize_t package_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct task_struct* process;
	struct mm_struct* mm;
	struct vm_area_struct* vm_area;
	struct file* file;
	int i, ii;
	char tmp[PACKAGE_SHOW_MAX_LEN + 1];
	char bf[PACKAGE_SHOW_MAX_LEN + 1];
	char cf[PACKAGE_SHOW_MAX_LEN + 1];
	char df[PACKAGE_SHOW_MAX_LEN + 1];
	ssize_t ret = 0;
	int state0 = 0;
	int state1 = 0;
	char* p;

	memset(tmp, 0, PACKAGE_SHOW_MAX_LEN + 1);
	memset(bf, 0, PACKAGE_SHOW_MAX_LEN + 1);
	memset(cf, 0, PACKAGE_SHOW_MAX_LEN + 1);

	if(package_held_pid != -1)
	{
		sphinx_printk("held pid 2: %d\n", package_held_pid);

		process = find_task_by_vpid(package_held_pid);

		if(process)
		{
			mm = process->mm;
			vm_area = mm->mmap;
		
			for(i = 0, ii = 0; i < mm->map_count; i++)
			{
				file = vm_area->vm_file;
		
				if(file)
				{
					p = d_path(&file->f_path, bf, PACKAGE_SHOW_MAX_LEN);
					
				  if(p == NULL || (long)p == ENAMETOOLONG)continue;
					
					if(vm_area->vm_flags & VM_MAYSHARE)
					{
						sphinx_printk("[s] : [%s][%08x]\n", p, (unsigned int)vm_area->vm_flags);

						switch(state0)
						{
							case 2:
							{
								if(strcmp(p, "/system/framework/core.jar") == 0)
								{
									state0 = 3;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}
	
							case 3:
							{
								if(	(strcmp(p, "/data/dalvik-cache/system@framework@core.jar@classes.dex") == 0) ||
									(strcmp(p, "/system/framework/core.odex") == 0))
								{
									state0 = 4;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}

							case 4:
							{
								if(strcmp(p, "/system/framework/ext.jar") == 0)
								{
									state0 = 5;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}
	
							case 5:
							{
								if(	(strcmp(p, "/data/dalvik-cache/system@framework@ext.jar@classes.dex") == 0) ||
									(strcmp(p, "/system/framework/ext.odex") == 0))
								{
									state0 = 6;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}

							case 6:
							{
								if(strcmp(p, "/system/framework/framework.jar") == 0)
								{
									state0 = 7;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}
	
							case 7:
							{
								if(	(strcmp(p, "/data/dalvik-cache/system@framework@framework.jar@classes.dex") == 0) ||
									(strcmp(p, "/system/framework/framework.odex") == 0))
								{
									state0 = 8;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}

							case 8:
							{
								if(strcmp(p, "/system/framework/android.policy.jar") == 0)
								{
									state0 = 9;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}
	
							case 9:
							{
								if(	(strcmp(p, "/data/dalvik-cache/system@framework@android.policy.jar@classes.dex") == 0) ||
									(strcmp(p, "/system/framework/android.policy.odex") == 0))
								{
									state0 = 10;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}

							case 10:
							{
								if(strcmp(p, "/system/framework/services.jar") == 0)
								{
									state0 = 11;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}
	
							case 11:
							{
								if(	(strcmp(p, "/data/dalvik-cache/system@framework@services.jar@classes.dex") == 0) ||
									(strcmp(p, "/system/framework/services.odex") == 0))
								{
									state0 = 12;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}
						}

						if(strstr(p, "/system/app/")&&strstr(p, ".apk"))
						{
							memset(tmp, 0, PACKAGE_SHOW_MAX_LEN + 1);
							snprintf(tmp, PACKAGE_SHOW_MAX_LEN - 1, "%s", p);
	
							sphinx_printk("%s\n", tmp);
							sphinx_printk("state1 -> %d\n", state1);
						}

						if(strstr(p, "/data/app/")&&strstr(p, ".apk"))
						{
							memset(tmp, 0, PACKAGE_SHOW_MAX_LEN + 1);
							snprintf(tmp, PACKAGE_SHOW_MAX_LEN -1, "%s", p);
	
							sphinx_printk("%s\n", tmp);
							sphinx_printk("state1 -> %d\n", state1);
						}

						if(strstr(p, "/data/app-private/")&&strstr(p, ".apk"))
						{
							memset(tmp, 0, PACKAGE_SHOW_MAX_LEN + 1);
							snprintf(tmp, PACKAGE_SHOW_MAX_LEN - 1, "%s", p);
	
							sphinx_printk("%s\n", tmp);
							sphinx_printk("state1 -> %d\n", state1);
						}

						switch(state1)
						{
							case 0:
							{
								int j;

								memset(cf, 0, PACKAGE_SHOW_MAX_LEN + 1);
								memcpy(cf, tmp, PACKAGE_SHOW_MAX_LEN - 2);

								j = strlen(cf);
								
								if(j > 4)
								{
									if(	cf[j-4] == '.' &&
										cf[j-3] == 'a' &&
										cf[j-2] == 'p' &&
										cf[j-1] == 'k'
									)
									{
										cf[j-3] = 'o';
										cf[j-2] = 'd';
										cf[j-1] = 'e';
										cf[j]   = 'x';

										if(strcmp(p, cf) == 0)
										{
											state1 = 1;
											sphinx_printk("%s\n", cf);
											sphinx_printk("state1 -> %d\n", state1);

											break;
										}
									}
								}

								memset(cf, 0, PACKAGE_SHOW_MAX_LEN + 1);
								memcpy(cf, tmp + 1, PACKAGE_SHOW_MAX_LEN - 1);

								for(j = 0; j < PACKAGE_SHOW_MAX_LEN; j++)
								{
									if(cf[j] == '/')cf[j] = '@';
								}

								snprintf(df, PACKAGE_SHOW_MAX_LEN - 1, "/data/dalvik-cache/%s@classes.dex", cf);

								if(strcmp(p, df) == 0)
								{
									state1 = 1;
									sphinx_printk("%s\n", df);
									sphinx_printk("state1 -> %d\n", state1);
								}

								break;
							}


						}
					}
					else
					{
						sphinx_printk("[-] : [%s][%08x]\n", p, (unsigned int)vm_area->vm_flags);

						switch(state0)
						{
							case 0:
							{
								if(strcmp(p, "/system/bin/app_process") == 0)
								{
									state0 = 1;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}

							case 1:
							{
								if(strcmp(p, "/system/bin/app_process") == 0)
								{
									state0 = 2;
									sphinx_printk("state0 -> %d\n", state0);
								}

								break;
							}
						}

						if(ii == 0 && state0 == 0 && strstr(p, "/system/bin") == p)
						{
							ret = strlen(p);

							sprintf(buf, "%s\n", p);

							state0 = 100;
							state1 = 100;

							break;
						}
					}

					if(state0 == 12 && state1 == 1)
					{
						ret = strlen(tmp);

						sprintf(buf, "%s", tmp);

						sphinx_printk("finally [%s]\n", tmp);

						state0 = 100;
						state1 = 100;
					}

					/* */

					if(strstr(p, "/data") == p && strstr(p, "/data/dalvik-cache") == NULL && strstr(p, "/data/app") == NULL && strstr(p, ".apk") == NULL)
					{
						sphinx_printk("refused !!\n");

						ret = 0;

						break;
					}

					ii++;
				}
		
				vm_area = vm_area->vm_next;
			}
		}

		package_held_pid = -1;
	}

	sphinx_printk("ret : %d\n", ret);

	return ret;
}

static ssize_t package_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	char tmp[PACKAGE_SHOW_MAX_LEN + 1];
	char* ptmp = &tmp[0];

	if(len > 32)return 0;
	if(package_held_pid != -1)return 0;
	
	memset(tmp, 0, PACKAGE_SHOW_MAX_LEN + 1);
	memcpy(tmp, buf, len);

	for(package_held_pid = 0; *ptmp != '\0' && *ptmp >= '0' && *ptmp <= '9'; ptmp++)
	{
		package_held_pid = 10 * package_held_pid + (*ptmp - '0');
	}

	sphinx_printk("held pid : %d\n", package_held_pid);

	return len;
}

static int sphinx_create_kobject(kobj_data *data)
{
	int ret;

	/*
	 * As we have a kset for this kobject, we need to set it before calling
	 * the kobject core.
	 */
	data->kobj->kset = sphinx_kset;

	/*
	 * Initialize and add the kobject to the kernel.  All the default files
	 * will be created here.  As we have already specified a kset for this
	 * kobject, we don't have to set a parent for the kobject, the kobject
	 * will be placed beneath that kset automatically.
	 */
	ret = kobject_init_and_add(data->kobj, data->ktype, NULL, "%s", data->name);
	if(ret) kobject_put(data->kobj);

	/*
	 * We are always responsible for sending the uevent that the kobject
	 * was added to the system.
	 */
	if(!ret) kobject_uevent(data->kobj, KOBJ_ADD);

	return ret;
}

static int __init sphinx_init(void)
{
	int ret;

	/* Create a simple kobject with the name of "sphinx" located under /sys/kernel/ */
	sphinx_kset = kset_create_and_add("digestmanager", NULL, kernel_kobj);
	if(!sphinx_kset) return -ENOMEM;

	ret = sphinx_create_kobject(&elfloader_data);
	ret = sphinx_create_kobject(&dexloader_data);
	ret = sphinx_create_kobject(&provenprocess_data);
	ret = sphinx_create_kobject(&secure_data);
	ret = sphinx_create_kobject(&package_data);

	return ret;
}

static void __exit sphinx_exit(void)
{
	kset_unregister(sphinx_kset);
}

module_init(sphinx_init);
module_exit(sphinx_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("shibata");
