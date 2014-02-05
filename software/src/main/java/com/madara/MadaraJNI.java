/*********************************************************************
 * Usage of this software requires acceptance of the SMASH-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/smash-cmu/wiki/License
 *********************************************************************/
package com.madara;




/**
 * Abstract class that insures loading of libMADARA.so and holds the C pointer
 */
public abstract class MadaraJNI
{	
	static
	{
		System.loadLibrary("ACE");
		System.loadLibrary("MADARA");
	}
	
	/**
	 * C pointer to an object
	 */
	private long cptr = 0;

	/**
	 * Set the C pointer to the object
	 * @param cptr C Pointer
	 */
	protected void setCPtr(long cptr)
	{
		this.cptr = cptr;
	}
	
	
	/**
	 * @return The C pointer of this object for passing to JNI functions
	 */
	protected long getCPtr()
	{
		return cptr;
	}
	
	
	/**
	 * @see java.lang.Object#toString()
	 * @return &lt;ClassName&gt;[&lt;C Pointer&gt;]
	 */
	public String toString()
	{
		return getClass().getName() + "[" + cptr + "]";
	}
	
}
