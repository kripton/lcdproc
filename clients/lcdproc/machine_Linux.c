
#ifdef linux

#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/param.h>
#include <sys/utsname.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <errno.h>

#ifdef USE_GETLOADAVG
# include <sys/loadavg.h>
#endif

#ifdef HAVE_PROCFS_H
# include <procfs.h>
#endif

#ifdef HAVE_SYS_PROCFS_H
# include <sys/procfs.h>
#endif

#include "main.h"
#include "mode.h"
#include "machine.h"
#include "config.h"
#include "shared/LL.h"

#define MAX_CPUS 8

static int batt_fd;
static int load_fd;
static int loadavg_fd;
static int meminfo_fd;
static int uptime_fd;

static char procbuf[1024]; //TODO ugly hack!

static FILE *mtab_fd;

int machine_init()
{
	uptime_fd	= -1;
	batt_fd		= -1;
	load_fd		= -1;
	loadavg_fd	= -1;
	meminfo_fd	= -1;

	if(uptime_fd == -1)
	{
		uptime_fd = open("/proc/uptime", O_RDONLY);
		if(uptime_fd < 0)
		{
			perror("open /proc/uptime");
			return(FALSE);
		}
	}

	if(load_fd == -1)
	{
		load_fd = open("/proc/stat", O_RDONLY);
		if(load_fd < 0)
		{
			perror("open /proc/stat");
			return(FALSE);
		}
	}

#ifndef USE_GETLOADAVG
	if(loadavg_fd == -1)
	{
		loadavg_fd = open("/proc/loadavg", O_RDONLY);
		if(loadavg_fd < 0)
		{
			perror("open /proc/loadavg");
			return(FALSE);
		}
	}
#endif

	if(meminfo_fd == -1)
	{
		meminfo_fd = open("/proc/meminfo", O_RDONLY);
		if(meminfo_fd < 0)
		{
			perror("open /proc/meminfo");
			return(FALSE);
		}
	}

	if(batt_fd == -1)
	{
		batt_fd = open("/proc/apm", O_RDONLY);
		if(batt_fd < 0)
		{
			//perror("open /proc/apm");
			batt_fd = -1;
		}
	}

	return(TRUE);
}

int machine_close()
{
	if(batt_fd != -1)
		close(batt_fd);
	batt_fd = -1;

	if(load_fd != -1)
		close(load_fd);
	load_fd = -1;

#ifndef USE_GETLOADAVG
	if(loadavg_fd != -1)
		close(loadavg_fd);
	loadavg_fd = -1;
#endif

	if(meminfo_fd != -1)
		close(meminfo_fd);
	meminfo_fd = -1;

	if(uptime_fd != -1)
		close(uptime_fd);
	uptime_fd = -1;

	return(TRUE);
}

static void
reread (int f, char *errmsg)
{
	if (lseek (f, 0L, 0) == 0 && read (f, procbuf, sizeof(procbuf) - 1) > 0)
		return;
	perror (errmsg);
	exit (1);
}

int
getentry (const char *tag, const char *bufptr)
{
	char *tail;
	int retval, len = strlen (tag);

	while (bufptr) {
		if (*bufptr == '\n')
			bufptr++;
		if (!strncmp (tag, bufptr, len)) {
			retval = strtol (bufptr + len, &tail, 10);
			if (tail == bufptr + len)
				return -1;
			else
				return retval;
		}
		bufptr = strchr (bufptr, '\n');
	}
	return -1;
}

int machine_get_battstat(int *acstat, int *battflag, int *percent)
{
	char str[64];
	int battstat;

	if(batt_fd == -1)
	{
		*acstat   = LCDP_AC_ON;
		*battflag = LCDP_BATT_ABSENT;
		*percent  = 100;
		return(TRUE);
	}

	if(lseek(batt_fd, 0, 0) != 0)
		return(FALSE);

	if(read(batt_fd, str, sizeof(str) - 1) < 0)
		return(FALSE);

	if(3 > sscanf(str + 13, "0x%x 0x%x 0x%x %d", acstat, &battstat, battflag, percent))
		return(FALSE);

	if(*battflag == 0xff)
		*battflag = LCDP_BATT_UNKNOWN;
	else
	{
		if(*battflag & 1)
			*battflag = LCDP_BATT_HIGH;
		if(*battflag & 2)
			*battflag = LCDP_BATT_LOW;
		if(*battflag & 4)
			*battflag = LCDP_BATT_CRITICAL;
		if(*battflag & 8 || battstat == 3)
			*battflag = LCDP_BATT_CHARGING;
		if(*battflag & 128)
			*battflag = LCDP_BATT_ABSENT;
	}

	switch(*acstat)
	{
		case 0:
			*acstat = LCDP_AC_OFF;
			break;
		case 1:
			*acstat = LCDP_AC_ON;
			break;
		case 2:
			*acstat = LCDP_AC_BACKUP;
			break;
		default:
			*acstat = LCDP_AC_UNKNOWN;
			break;
	}

	return(TRUE);
}

int machine_get_fs(mounts_type fs[], int *cnt)
{
#ifdef STAT_STATVFS
	struct statvfs fsinfo;
#else
	struct statfs fsinfo;
#endif
	char line[256];
	int x = 0, y;

#ifdef MTAB_FILE
	mtab_fd = fopen(MTAB_FILE, "r");
#else
#error "Can't find your mounted filesystem table file."
#endif

	// Get rid of old, unmounted filesystems...
	memset(fs, 0, sizeof(mounts_type) * 256);

	while(x < 256)
	{
		if(fgets(line, 256, mtab_fd) == NULL)
		{
			fclose (mtab_fd);
			*cnt = x;
			return(FALSE);
		}

		sscanf(line, "%s %s %s", fs[x].dev, fs[x].mpoint, fs[x].type);

		if (strcmp(fs[x].type, "proc")
			&& strcmp(fs[x].type, "tmpfs")
#ifndef STAT_NFS
			 && strcmp(fs[x].type, "nfs")
#endif
#ifndef STAT_SMBFS
			 && strcmp(fs[x].type, "smbfs")
#endif
			 )
		{
#ifdef STAT_STATVFS
			y = statvfs(fs[x].mpoint, &fsinfo);
#elif STAT_STATFS2_BSIZE
			y = statfs(fs[x].mpoint, &fsinfo);
#elif STAT_STATFS4
			y = statfs(fs[x].mpoint, &fsinfo, sizeof (fsinfo), 0);
#else
#error "statfs for this system not yet supported"
#endif

			fs[x].blocks = fsinfo.f_blocks;
			if(fs[x].blocks > 0)
			{
				fs[x].bsize = fsinfo.f_bsize;
				fs[x].bfree = fsinfo.f_bfree;
				fs[x].files = fsinfo.f_files;
				fs[x].ffree = fsinfo.f_ffree;
				x++;
			}
		}
	}

	fclose(mtab_fd);
	*cnt = x;
	return(TRUE);
}

int machine_get_load(load_type *curr_load)
{
	static load_type last_load = { 0, 0, 0, 0, 0 };
	load_type load;

	reread(load_fd, "get_load:");
	sscanf(procbuf, "%*s %lu %lu %lu %lu\n", &load.user, &load.nice, &load.system, &load.idle);
	load.total = load.user + load.nice + load.system + load.idle;

	curr_load->user   = load.user   - last_load.user;
	curr_load->nice   = load.nice   - last_load.nice;
	curr_load->system = load.system - last_load.system;
	curr_load->idle   = load.idle   - last_load.idle;
	curr_load->total  = load.total  - last_load.total;
	last_load = load;

	return(TRUE);
}

int machine_get_loadavg(double *load)
{
#ifdef USE_GETLOADAVG
	double loadavg[LOADAVG_NSTATS];

	if(getloadavg(loadavg, LOADAVG_NSTATS) < 0)
	{
		perror("getloadavg");
		*load = 1.;
		return(FALSE);
	}
	*load = loadavg[LOADAVG_1MIN];
#else
	reread(loadavg_fd, "get_load:");
	sscanf(procbuf, "%lf", load);
#endif
	return(TRUE);
}

int machine_get_meminfo(meminfo_type *result)
{
	reread(meminfo_fd, "get_meminfo:");
	result[0].total   = getentry("MemTotal:", procbuf);
	result[0].free    = getentry("MemFree:", procbuf);
	result[0].shared  = getentry("MemShared:", procbuf);
	result[0].buffers = getentry("Buffers:", procbuf);
	result[0].cache   = getentry("Cached:", procbuf);
	result[1].total   = getentry("SwapTotal:", procbuf);
	result[1].free    = getentry("SwapFree:", procbuf);

	return(TRUE);
}

int machine_get_procs(LinkedList *procs)
{
	// Much of this code was ripped from "gmemusage"
	char buf[128];
	DIR *proc;
	FILE *StatusFile;
	struct dirent *procdir;
	procinfo_type *p;

	char	procName[16];
	int		procSize, procRSS, procData, procStk, procExe;
	const char
			*NameLine	= "Name:",
			*VmSizeLine	= "VmSize:",
			*VmRSSLine	= "VmRSS",
			*VmDataLine	= "VmData",
			*VmStkLine	= "VmStk",
			*VmExeLine	= "VmExe";
	const int
			NameLineLen		= strlen(NameLine),
			VmSizeLineLen	= strlen(VmSizeLine),
			VmDataLineLen	= strlen(VmDataLine),
			VmStkLineLen	= strlen(VmStkLine),
			VmExeLineLen	= strlen(VmExeLine),
			VmRSSLineLen	= strlen(VmRSSLine);
	int threshold = 400, unique;

	if((proc = opendir("/proc")) == NULL)
	{
		perror("mem_top_screen: unable to open /proc");
		return(FALSE);
	}

	while((procdir = readdir(proc)))
	{
		if(!index("1234567890", procdir->d_name[0]))
			continue;

		sprintf(buf, "/proc/%s/status", procdir->d_name);
		if((StatusFile = fopen(buf, "r")) == NULL)
		{
			// Not a serious error; process has finished before we could
			// examine it:
			continue;
		}

		procRSS = procSize = procData = procStk = procExe = 0;
		while(fgets(buf, sizeof(buf), StatusFile))
		{
			if(!strncmp(buf, NameLine, NameLineLen))
			{
				/* Name: procName */
				sscanf(buf, "%*s %s", procName);
			} else if(!strncmp(buf, VmSizeLine, VmSizeLineLen))
			{
				/* VmSize: procSize kB */
				sscanf(buf, "%*s %d", &procSize);
			} else if(!strncmp (buf, VmRSSLine, VmRSSLineLen))
			{
				/* VmRSS: procRSS kB */
				sscanf(buf, "%*s %d", &procRSS);
			} else if(!strncmp(buf, VmDataLine, VmDataLineLen))
			{
				/* VmData: procData kB */
				sscanf(buf, "%*s %d", &procData);
			} else if(!strncmp(buf, VmStkLine, VmStkLineLen))
			{
				/* VmStk: procStk kB */
				sscanf(buf, "%*s %d", &procStk);
			} else if(!strncmp(buf, VmExeLine, VmExeLineLen))
			{
				/* VmExe: procExe kB */
				sscanf(buf, "%*s %d", &procExe);
			}
		}
		fclose(StatusFile);

		if(procSize > threshold)
		{
			// Figure out if it's sharing any memory...
			unique = 1;
			LL_Rewind(procs);
			do
			{
				p = LL_Get (procs);
				if(p)
				{
					if(0 == strcmp(p->name, procName))
					{
						unique = 0;
						p->number++;
						p->totl += procData + procStk + procExe;
					}
				}
			} while(LL_Next(procs) == 0);

			// If this is the first one by this name...
			if(unique)
			{
				p = malloc(sizeof(procinfo_type));
				if(!p)
				{
					perror("mem_top_screen: Error allocating process entry");
					break;
				}
				strcpy(p->name, procName);
				p->totl = procData + procStk + procExe;
				p->number = 1;
				// TODO:  Check for errors here?
				LL_Push(procs, (void *) p);
			}
		}
	}
	closedir(proc);

	return(TRUE);
}

int machine_get_smpload(load_type *result, int *numcpus)
{
	char *token;
	static load_type last_load[MAX_CPUS];
	load_type curr_load[MAX_CPUS];

	*numcpus = 0;

	reread(load_fd, "get_load");

	// Look for lines starting with "cpu0", "cpu1", etc.
	token = strtok(procbuf, "\n");
	while(token)
	{
		if((strlen(token) > 3) && (!strncmp(token, "cpu", 3)) && isdigit(token[3]))
		{
			sscanf(token, "%*s %lu %lu %lu %lu", &curr_load[*numcpus].user, &curr_load[*numcpus].nice, &curr_load[*numcpus].system, &curr_load[*numcpus].idle);

			curr_load[*numcpus].total = curr_load[*numcpus].user + curr_load[*numcpus].nice + curr_load[*numcpus].system + curr_load[*numcpus].idle;
			result[*numcpus].total	= curr_load[*numcpus].total - last_load[*numcpus].total;
			result[*numcpus].user	= curr_load[*numcpus].user - last_load[*numcpus].user;
			result[*numcpus].nice	= curr_load[*numcpus].nice - last_load[*numcpus].nice;
			result[*numcpus].system	= curr_load[*numcpus].system - last_load[*numcpus].system;
			result[*numcpus].idle	= curr_load[*numcpus].idle - last_load[*numcpus].idle;
			last_load[*numcpus].total	= curr_load[*numcpus].total;
			last_load[*numcpus].user	= curr_load[*numcpus].user;
			last_load[*numcpus].nice	= curr_load[*numcpus].nice;
			last_load[*numcpus].system	= curr_load[*numcpus].system;
			last_load[*numcpus].idle	= curr_load[*numcpus].idle;

			(*numcpus)++;
		}
		token = strtok(NULL, "\n");
	}
	return(TRUE);
}

int machine_get_uptime(double *up, double *idle)
{
	double local_up, local_idle;
	
	reread(uptime_fd, "get_uptime:");
	sscanf(procbuf, "%lf %lf", &local_up, &local_idle);
	if (up != NULL)
			*up = local_up;
	if (idle != NULL)
			*idle = (local_up != 0)
				? 100 * local_idle / local_up
				: 100;

	return(TRUE);
}

#endif /* linux */