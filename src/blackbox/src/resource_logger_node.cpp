// resource_logger — 1 Hz polling host resource → blackbox::resource (4 files)
//   global.bin     CPU/RAM/GPU/throttle/net/wifi/TCP (GlobalRecord 424B)
//   image_proc.bin  per-process CPU/RSS for image driver  (ProcRecord 48B)
//   slam_proc.bin   per-process CPU/RSS for fast_livo2    (ProcRecord 48B)
//   livox_proc.bin  per-process CPU/RSS for livox driver  (ProcRecord 48B)
//
// Spec: references/blackbox/spec/resource.md (확정 ABI + 함수 계약)
// Sources/units: references/blackbox/findings/resource/measurements.md (실측 2026-05-21)

// C system headers
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// C++ standard library
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// Other libraries
#include <rclcpp/rclcpp.hpp>

// Project
#include "blackbox/blackbox.hpp"

namespace
{

constexpr size_t kBufCap = 8192;          // /proc/stat ~1357B, /proc/net/dev ~few KB

// ─────────────────────────────────────────────────────────────
// parse_* — pure functions, no allocation, no out-of-bound access.
// ─────────────────────────────────────────────────────────────

struct CpuStat
{
  uint64_t total[8];
  uint64_t idle[8];
  bool     ok;
};

CpuStat parse_cpu_stat(const char * buf, size_t /*n*/)
{
  CpuStat r{};
  if (buf == nullptr) {
    return r;
  }
  for (int core = 0; core < 8; ++core) {
    char prefix[8];
    std::snprintf(prefix, sizeof(prefix), "cpu%d ", core);
    const char * p = std::strstr(buf, prefix);
    if (p == nullptr) {
      return r;
    }
    p += std::strlen(prefix);
    uint64_t fields[10] = {0};
    char * end;
    for (int i = 0; i < 10; ++i) {
      fields[i] = std::strtoull(p, &end, 10);
      if (end == p) {
        return r;
      }
      p = end;
    }
    uint64_t total = 0;
    for (int i = 0; i < 10; ++i) {
      total += fields[i];
    }
    r.total[core] = total;
    r.idle[core]  = fields[3];
  }
  r.ok = true;
  return r;
}

struct MemInfo
{
  uint64_t total_kb;
  uint64_t avail_kb;
  bool     ok;
};

MemInfo parse_meminfo(const char * buf, size_t /*n*/)
{
  MemInfo r{};
  if (buf == nullptr) {
    return r;
  }
  auto find_kv = [&](const char * key) -> uint64_t {
    const char * p = std::strstr(buf, key);
    if (p == nullptr) {
      return 0;
    }
    p += std::strlen(key);
    return std::strtoull(p, nullptr, 10);
  };
  r.total_kb = find_kv("MemTotal:");
  r.avail_kb = find_kv("MemAvailable:");
  r.ok = (r.total_kb > 0 && r.avail_kb > 0);
  return r;
}

struct U32
{
  uint32_t v;
  bool     ok;
};

U32 parse_u32_line(const char * buf, size_t n)
{
  U32 r{};
  if (buf == nullptr || n == 0) {
    return r;
  }
  char * end;
  unsigned long v = std::strtoul(buf, &end, 10);
  if (end == buf) {
    return r;
  }
  r.v  = static_cast<uint32_t>(v);
  r.ok = true;
  return r;
}

struct NetDev
{
  blackbox::NetSlot net[4];
  bool              ok;
};

NetDev parse_net_dev(const char * buf, size_t /*n*/,
                     const std::vector<std::string> & iface_filter)
{
  NetDev r{};
  if (buf == nullptr) {
    return r;
  }
  for (size_t slot = 0; slot < 4; ++slot) {
    if (slot >= iface_filter.size()) {
      break;
    }
    std::strncpy(r.net[slot].iface, iface_filter[slot].c_str(), 15);
    r.net[slot].iface[15] = '\0';
    std::string key = iface_filter[slot] + ":";
    const char * p = std::strstr(buf, key.c_str());
    if (p == nullptr) {
      continue;
    }
    p += key.size();
    uint64_t fields[16] = {0};
    char * end;
    int got = 0;
    for (int i = 0; i < 16; ++i) {
      fields[i] = std::strtoull(p, &end, 10);
      if (end == p) {
        break;
      }
      p = end;
      ++got;
    }
    if (got < 12) {
      continue;
    }
    r.net[slot].rx_bytes = fields[0];
    r.net[slot].rx_drop  = fields[3];
    r.net[slot].tx_bytes = fields[8];
    r.net[slot].tx_drop  = fields[11];
  }
  r.ok = true;
  return r;
}

struct Wireless
{
  uint32_t link;
  int32_t  level;
  bool     ok;
};

Wireless parse_wireless(const char * buf, size_t n)
{
  Wireless r{};
  if (buf == nullptr || n == 0) {
    return r;
  }
  // Skip header lines (contain '|'); pick first data line with ':'.
  size_t i = 0;
  while (i < n) {
    size_t lend = i;
    while (lend < n && buf[lend] != '\n') {
      ++lend;
    }
    size_t llen = lend - i;
    const char * line = buf + i;
    if (std::memchr(line, '|', llen) == nullptr) {
      const char * col = static_cast<const char *>(std::memchr(line, ':', llen));
      if (col != nullptr) {
        const char * q   = col + 1;
        const char * eol = line + llen;
        while (q < eol && *q == ' ') {
          ++q;
        }
        while (q < eol && *q != ' ') {   // skip status token
          ++q;
        }
        while (q < eol && *q == ' ') {
          ++q;
        }
        char * end;
        double link_d = std::strtod(q, &end);
        if (end != q) {
          while (end < eol && (*end == ' ' || *end == '.')) {
            ++end;
          }
          double level_d = std::strtod(end, nullptr);
          if (link_d > 0) {
            r.link  = static_cast<uint32_t>(link_d);
            r.level = static_cast<int32_t>(level_d);
            r.ok    = true;
            return r;
          }
        }
      }
    }
    i = lend + 1;
  }
  return r;
}

struct SnmpTcp
{
  uint64_t out;
  uint64_t retrans;
  bool     ok;
};

SnmpTcp parse_snmp_tcp(const char * buf, size_t /*n*/)
{
  SnmpTcp r{};
  if (buf == nullptr) {
    return r;
  }
  const char * h = std::strstr(buf, "Tcp: ");
  if (h == nullptr) {
    return r;
  }
  const char * h_p   = h + 5;
  const char * h_eol = std::strchr(h_p, '\n');
  if (h_eol == nullptr) {
    return r;
  }
  int idx_out = -1;
  int idx_retrans = -1;
  int col = 0;
  const char * p = h_p;
  while (p < h_eol) {
    while (p < h_eol && *p == ' ') {
      ++p;
    }
    if (p >= h_eol) {
      break;
    }
    const char * t0 = p;
    while (p < h_eol && *p != ' ') {
      ++p;
    }
    size_t tlen = p - t0;
    if (tlen == 7 && std::memcmp(t0, "OutSegs", 7) == 0) {
      idx_out = col;
    } else if (tlen == 11 && std::memcmp(t0, "RetransSegs", 11) == 0) {
      idx_retrans = col;
    }
    ++col;
  }
  if (idx_out < 0 || idx_retrans < 0) {
    return r;
  }
  const char * v = std::strstr(h_eol, "Tcp: ");
  if (v == nullptr) {
    return r;
  }
  v += 5;
  char * end;
  int max_idx = std::max(idx_out, idx_retrans);
  for (int i = 0; i <= max_idx; ++i) {
    while (*v == ' ') {
      ++v;
    }
    uint64_t val = std::strtoull(v, &end, 10);
    if (end == v) {
      return r;
    }
    if (i == idx_out) {
      r.out = val;
    }
    if (i == idx_retrans) {
      r.retrans = val;
    }
    v = end;
  }
  r.ok = true;
  return r;
}

struct ProcStat
{
  uint64_t utime;
  uint64_t stime;
  bool     ok;
};

ProcStat parse_proc_stat(const char * buf, size_t n)
{
  ProcStat r{};
  if (buf == nullptr || n == 0) {
    return r;
  }
  // comm can contain spaces/parens → use rindex(')').
  const char * close = nullptr;
  for (const char * p = buf + n - 1; p > buf; --p) {
    if (*p == ')') {
      close = p;
      break;
    }
  }
  if (close == nullptr) {
    return r;
  }
  const char * p = close + 1;
  // Fields after ')': state ppid pgrp session tty_nr tpgid flags
  // minflt cminflt majflt cmajflt utime stime ...  (utime=idx 11, stime=12 after ')')
  for (int i = 0; i < 13; ++i) {
    while (*p == ' ') {
      ++p;
    }
    if (i == 0) {
      ++p;  // single-char state
    } else {
      char * e;
      uint64_t v = std::strtoull(p, &e, 10);
      if (e == p) {
        return r;
      }
      if (i == 11) {
        r.utime = v;
      } else if (i == 12) {
        r.stime = v;
      }
      p = e;
    }
  }
  r.ok = true;
  return r;
}

struct ProcStatm
{
  uint64_t rss_bytes;
  bool     ok;
};

ProcStatm parse_proc_statm(const char * buf, size_t /*n*/)
{
  ProcStatm r{};
  if (buf == nullptr) {
    return r;
  }
  char * end;
  (void)std::strtoull(buf, &end, 10);     // size (skip)
  if (end == buf) {
    return r;
  }
  while (*end == ' ') {
    ++end;
  }
  uint64_t resident = std::strtoull(end, &end, 10);
  if (end == nullptr) {
    return r;
  }
  r.rss_bytes = resident * static_cast<uint64_t>(::sysconf(_SC_PAGESIZE));
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────
// pread wrapper — cached fd, fixed buf.
// ─────────────────────────────────────────────────────────────
ssize_t pread_buf(int fd, char * buf, size_t cap)
{
  if (fd < 0) {
    return -1;
  }
  ssize_t n = ::pread(fd, buf, cap - 1, 0);
  if (n < 0) {
    return n;
  }
  buf[n] = '\0';
  return n;
}

}  // namespace

// ─────────────────────────────────────────────────────────────
// ResourceSampler — owns fds, performs sample_once() each tick.
// ─────────────────────────────────────────────────────────────
class ResourceSampler
{
public:
  void init(const std::string & dir,
            const std::vector<std::string> & proc_prefixes,
            const std::vector<std::string> & net_ifaces,
            size_t max_scan_pids)
  {
    dir_           = dir;
    proc_prefixes_ = proc_prefixes;
    net_ifaces_    = net_ifaces;
    max_scan_pids_ = max_scan_pids;
    procs_.assign(proc_prefixes_.size(), ProcEntry{});

    fd_stat_     = ::open("/proc/stat",                                            O_RDONLY);
    fd_meminfo_  = ::open("/proc/meminfo",                                         O_RDONLY);
    fd_gpu_load_ = ::open("/sys/devices/platform/gpu.0/load",                      O_RDONLY);
    fd_gpu_freq_ = ::open("/sys/class/devfreq/17000000.gpu/cur_freq",              O_RDONLY);
    fd_net_      = ::open("/proc/net/dev",                                         O_RDONLY);
    fd_wl_       = ::open("/proc/net/wireless",                                    O_RDONLY);
    fd_snmp_     = ::open("/proc/net/snmp",                                        O_RDONLY);
    for (int c = 0; c < 8; ++c) {
      char path[96];
      std::snprintf(path, sizeof(path),
                    "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", c);
      fd_cpufreq_[c] = ::open(path, O_RDONLY);
    }
    for (int i = 0; i < 3; ++i) {
      fd_thr_[i] = -1;
    }
    open_throttle_devs();

    discover_procs();
    blackbox::resource::init(dir_);
  }

  void sample_once()
  {
    const uint64_t t_sample = blackbox::mono_raw_ns();
    char buf[kBufCap];

    blackbox::GlobalRecord g{};

    // [1] CPU per-core (raw jiffies)
    ssize_t n = pread_buf(fd_stat_, buf, kBufCap);
    if (n > 0) {
      auto p = parse_cpu_stat(buf, static_cast<size_t>(n));
      if (p.ok) {
        std::memcpy(g.cpu_total, p.total, sizeof(g.cpu_total));
        std::memcpy(g.cpu_idle,  p.idle,  sizeof(g.cpu_idle));
        g.valid |= blackbox::V_CPU;
      }
    }
    // [2] RAM
    n = pread_buf(fd_meminfo_, buf, kBufCap);
    if (n > 0) {
      auto m = parse_meminfo(buf, static_cast<size_t>(n));
      if (m.ok) {
        g.mem_total_kb = m.total_kb;
        g.mem_avail_kb = m.avail_kb;
        g.valid |= blackbox::V_MEM;
      }
    }
    // [5] CPU freq per-core
    {
      bool all_ok = true;
      for (int c = 0; c < 8; ++c) {
        n = pread_buf(fd_cpufreq_[c], buf, kBufCap);
        if (n <= 0) {
          all_ok = false;
          continue;
        }
        auto u = parse_u32_line(buf, static_cast<size_t>(n));
        if (!u.ok) {
          all_ok = false;
          continue;
        }
        g.cpu_freq[c] = u.v;
      }
      if (all_ok) {
        g.valid |= blackbox::V_CPUFREQ;
      }
    }
    // [3]+[4] GPU
    bool gpu_ok = true;
    n = pread_buf(fd_gpu_load_, buf, kBufCap);
    if (n > 0) {
      auto u = parse_u32_line(buf, n);
      if (u.ok) {
        g.gpu_load = u.v;
      } else {
        gpu_ok = false;
      }
    } else {
      gpu_ok = false;
    }
    n = pread_buf(fd_gpu_freq_, buf, kBufCap);
    if (n > 0) {
      auto u = parse_u32_line(buf, n);
      if (u.ok) {
        g.gpu_freq_hz = u.v;
      } else {
        gpu_ok = false;
      }
    } else {
      gpu_ok = false;
    }
    if (gpu_ok) {
      g.valid |= blackbox::V_GPU;
    }
    // [6] throttle (cooling cur_state)
    {
      uint8_t * dst[3] = {&g.thr_cpu0, &g.thr_cpu4, &g.thr_gpu};
      bool all_ok = true;
      for (int i = 0; i < 3; ++i) {
        n = pread_buf(fd_thr_[i], buf, kBufCap);
        if (n <= 0) {
          all_ok = false;
          continue;
        }
        auto u = parse_u32_line(buf, n);
        if (!u.ok) {
          all_ok = false;
          continue;
        }
        *dst[i] = static_cast<uint8_t>(u.v);
      }
      if (all_ok) {
        g.valid |= blackbox::V_THROTTLE;
      }
    }
    // [7] network
    n = pread_buf(fd_net_, buf, kBufCap);
    if (n > 0) {
      auto nd = parse_net_dev(buf, n, net_ifaces_);
      if (nd.ok) {
        std::memcpy(g.net, nd.net, sizeof(g.net));
        g.valid |= blackbox::V_NET;
      }
    }
    // [8] wifi
    n = pread_buf(fd_wl_, buf, kBufCap);
    if (n > 0) {
      auto w = parse_wireless(buf, n);
      if (w.ok) {
        g.wifi_link  = w.link;
        g.wifi_level = w.level;
        g.valid |= blackbox::V_WIFI;
      }
    }
    // [9] TCP
    n = pread_buf(fd_snmp_, buf, kBufCap);
    if (n > 0) {
      auto t = parse_snmp_tcp(buf, n);
      if (t.ok) {
        g.tcp_out     = t.out;
        g.tcp_retrans = t.retrans;
        g.valid |= blackbox::V_TCP;
      }
    }

    blackbox::resource::store_global(g, t_sample);

    // Per-process — fixed 3 slots matching ProcSlot enum
    for (size_t s = 0; s < procs_.size() && s < 3; ++s) {
      ProcEntry & e = procs_[s];
      if (e.pid == 0) {
        rediscover_slot(s);
        if (e.pid == 0) {
          continue;
        }
      }

      blackbox::ProcRecord p{};
      n = pread_buf(e.fd_stat, buf, kBufCap);
      auto ps = (n > 0) ? parse_proc_stat(buf, n) : ProcStat{};
      if (!ps.ok) {
        // pid likely gone — close fds, mark for rediscover next tick
        if (e.fd_stat >= 0) {
          ::close(e.fd_stat);
          e.fd_stat = -1;
        }
        if (e.fd_statm >= 0) {
          ::close(e.fd_statm);
          e.fd_statm = -1;
        }
        e.pid = 0;
        continue;
      }
      n = pread_buf(e.fd_statm, buf, kBufCap);
      auto pm = (n > 0) ? parse_proc_statm(buf, n) : ProcStatm{};
      if (!pm.ok) {
        continue;
      }

      p.pid       = e.pid;
      p.utime     = ps.utime;
      p.stime     = ps.stime;
      p.rss_bytes = pm.rss_bytes;
      blackbox::resource::store_proc(
        static_cast<blackbox::resource::ProcSlot>(s), p, t_sample);
    }
  }

  void shutdown()
  {
    blackbox::resource::shutdown();
    auto close_fd = [](int & fd) {
      if (fd >= 0) {
        ::close(fd);
        fd = -1;
      }
    };
    close_fd(fd_stat_);
    close_fd(fd_meminfo_);
    close_fd(fd_gpu_load_);
    close_fd(fd_gpu_freq_);
    close_fd(fd_net_);
    close_fd(fd_wl_);
    close_fd(fd_snmp_);
    for (int c = 0; c < 8; ++c) {
      close_fd(fd_cpufreq_[c]);
    }
    for (int i = 0; i < 3; ++i) {
      close_fd(fd_thr_[i]);
    }
    for (auto & e : procs_) {
      close_fd(e.fd_stat);
      close_fd(e.fd_statm);
      e.pid = 0;
    }
  }

private:
  struct ProcEntry
  {
    uint32_t pid;
    int      fd_stat;
    int      fd_statm;
  };

  void open_throttle_devs()
  {
    DIR * d = ::opendir("/sys/class/thermal");
    if (d == nullptr) {
      return;
    }
    const std::array<const char *, 3> wanted = {
      "cpufreq-cpu0", "cpufreq-cpu4", "devfreq-17000000.gpu"};
    struct dirent * ent;
    while ((ent = ::readdir(d)) != nullptr) {
      if (std::strncmp(ent->d_name, "cooling_device", 14) != 0) {
        continue;
      }
      char type_path[512];
      std::snprintf(type_path, sizeof(type_path),
                    "/sys/class/thermal/%s/type", ent->d_name);
      int tfd = ::open(type_path, O_RDONLY);
      if (tfd < 0) {
        continue;
      }
      char buf[64] = {0};
      ssize_t r = ::pread(tfd, buf, sizeof(buf) - 1, 0);
      ::close(tfd);
      if (r <= 0) {
        continue;
      }
      // trim trailing newline
      while (r > 0 && (buf[r - 1] == '\n' || buf[r - 1] == ' ')) {
        buf[--r] = '\0';
      }
      for (size_t i = 0; i < wanted.size(); ++i) {
        if (std::strcmp(buf, wanted[i]) == 0) {
          char cs_path[512];
          std::snprintf(cs_path, sizeof(cs_path),
                        "/sys/class/thermal/%s/cur_state", ent->d_name);
          fd_thr_[i] = ::open(cs_path, O_RDONLY);
          break;
        }
      }
    }
    ::closedir(d);
  }

  void discover_procs()
  {
    DIR * d = ::opendir("/proc");
    if (d == nullptr) {
      return;
    }
    struct dirent * ent;
    size_t scanned = 0;
    while ((ent = ::readdir(d)) != nullptr && scanned < max_scan_pids_) {
      const char * nm = ent->d_name;
      if (nm[0] < '0' || nm[0] > '9') {
        continue;
      }
      ++scanned;
      char exe_path[64];
      std::snprintf(exe_path, sizeof(exe_path), "/proc/%s/exe", nm);
      char link_target[512];
      ssize_t r = ::readlink(exe_path, link_target, sizeof(link_target) - 1);
      if (r <= 0) {
        continue;
      }
      link_target[r] = '\0';
      const char * slash = std::strrchr(link_target, '/');
      const char * base  = (slash != nullptr) ? slash + 1 : link_target;
      for (size_t s = 0; s < proc_prefixes_.size(); ++s) {
        if (procs_[s].pid != 0) {
          continue;
        }
        const std::string & pref = proc_prefixes_[s];
        if (std::strncmp(base, pref.c_str(), pref.size()) == 0) {
          uint32_t pid = static_cast<uint32_t>(std::strtoul(nm, nullptr, 10));
          char p_stat[64];
          char p_statm[64];
          std::snprintf(p_stat,  sizeof(p_stat),  "/proc/%u/stat",  pid);
          std::snprintf(p_statm, sizeof(p_statm), "/proc/%u/statm", pid);
          procs_[s].pid      = pid;
          procs_[s].fd_stat  = ::open(p_stat,  O_RDONLY);
          procs_[s].fd_statm = ::open(p_statm, O_RDONLY);
          break;
        }
      }
    }
    ::closedir(d);
  }

  void rediscover_slot(size_t slot)
  {
    if (slot >= procs_.size()) {
      return;
    }
    if (procs_[slot].pid != 0) {
      return;
    }
    discover_procs();   // refills any empty slot
  }

  std::string              dir_;
  std::vector<std::string> proc_prefixes_;
  std::vector<std::string> net_ifaces_;
  size_t                   max_scan_pids_{4096};
  int                      fd_stat_{-1};
  int                      fd_meminfo_{-1};
  int                      fd_gpu_load_{-1};
  int                      fd_gpu_freq_{-1};
  int                      fd_net_{-1};
  int                      fd_wl_{-1};
  int                      fd_snmp_{-1};
  int                      fd_cpufreq_[8]{};
  int                      fd_thr_[3]{};
  std::vector<ProcEntry>   procs_;
};

// ─────────────────────────────────────────────────────────────
// ResourceLogger — rclcpp::Node, wall_timer 1 Hz.
// ─────────────────────────────────────────────────────────────
class ResourceLogger : public rclcpp::Node
{
public:
  ResourceLogger()
  : Node("resource_logger")
  {
    const auto proc_prefixes = declare_parameter<std::vector<std::string>>(
      "proc_prefixes",
      std::vector<std::string>{
      "see3cam24cug", "fastlivo_mapping", "livox_ros_driver2"});
    const auto net_ifaces = declare_parameter<std::vector<std::string>>(
      "net_ifaces",
      std::vector<std::string>{"enP8p1s0", "wlP1p1s0", "tailscale0", "lo"});
    const auto log_dir   = declare_parameter<std::string>("log_dir", "");
    const auto period_s  = declare_parameter<double>("sample_period_s", 1.0);
    const auto max_scan  = declare_parameter<int>("max_scan_pids", 4096);

    const std::string dir = log_dir.empty() ? blackbox::log_dir() : log_dir;
    sampler_.init(dir, proc_prefixes, net_ifaces, static_cast<size_t>(max_scan));

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_s));
    timer_ = create_wall_timer(period, [this]() {sampler_.sample_once();});

    RCLCPP_INFO(get_logger(),
                "resource_logger up — dir=%s period=%.3fs prefixes=%zu ifaces=%zu",
                dir.c_str(), period_s, proc_prefixes.size(), net_ifaces.size());
  }

  ~ResourceLogger() override
  {
    sampler_.shutdown();
  }

private:
  ResourceSampler              sampler_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ResourceLogger>());
  rclcpp::shutdown();
  return 0;
}
