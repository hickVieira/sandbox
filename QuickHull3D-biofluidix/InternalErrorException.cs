/*
 * Copyright John E. Lloyd, 2004. All rights reserved. Permission to use,
 * copy, modify and redistribute is granted, provided that this copyright
 * notice is retained and the author is given credit whenever appropriate.
 *
 * This  software is distributed "as is", without any warranty, including 
 * any implied warranty of merchantability or fitness for a particular
 * use. The author assumes no responsibility for, and shall not be liable
 * for, any special, indirect, or consequential damages, or any damages
 * whatsoever, arising out of or in connection with the use of this
 * software.
 */

using System;

namespace QuickHull3D
{
    /// <summary>
    /// Exception thrown when QuickHull3D encounters an internal error.
    /// </summary>
    public class InternalErrorException : Exception
    {

        public InternalErrorException(string msg) : base(msg)
        {
        }
    }
}
