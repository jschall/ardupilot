mergeInto(LibraryManager.library, {
  __syscall_setsockopt: function(fd, level, optname, optval, optlen) {
    return 0;
  },
});
