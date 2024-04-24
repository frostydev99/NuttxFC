/****************************************************************************
 * apps/crypto/mbedtls/source/entropy_alt.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_hardware_poll(FAR void *data,
                          FAR unsigned char *output,
                          size_t len,
                          FAR size_t *olen)
{
  int fd;
  size_t read_len;
  *olen = 0;

  fd = open("/dev/random", O_RDONLY, 0);
  if (fd < 0)
    {
      return -errno;
    }

  read_len = read(fd, output, len);
  if (read_len != len)
    {
      close(fd);
      return -errno;
    }

  close(fd);
  *olen = len;

  return 0;
}
