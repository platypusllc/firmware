/*********************************************************************
 * Usage of this software requires acceptance of the SMASH-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/smash-cmu/wiki/License
 *********************************************************************/
package com.madara;

/**
 * Encapsulates settings for an evaluation statement.
 */
public class EvalSettings extends MadaraJNI
{
	
	//Constructors
	private static native long jni_evalSettings();
	private static native long jni_evalSettings(long oldPtr);
	
	//Getters/Setters
	private static native void jni_setDelaySendingModifieds(long cptr, boolean delaySendingModifieds);
	private static native boolean jni_getDelaySendingModifieds(long cptr);
	private static native void jni_setPrePrintStatement(long cptr, String prePrintStatement);
	private static native String jni_getPrePrintStatement(long cptr);
	private static native void jni_setPostPrintStatement(long cptr, String prePrintStatement);
	private static native String jni_getPostPrintStatement(long cptr);
	private static native void jni_setAlwaysOverwrite(long cptr, boolean alwaysOverwrite);
	private static native boolean jni_getAlwaysOverwrite(long cptr);
	private static native void jni_setTreatGlobalsAsLocals(long cptr, boolean treatGlobalsAsLocals);
	private static native boolean jni_getTreatGlobalsAsLocals(long cptr);
	private static native void jni_setClockIncrement(long cptr, long defaultClockIncrement);
	private static native long jni_getClockIncrement(long cptr);

	private static native void jni_freeEvalSettings(long cptr);

	public static final EvalSettings DEFAULT_EVAL_SETTINGS = new EvalSettings(jni_evalSettings());

	/**
	 * Used to determine if the current object should be allowed to change or not
	 */
	protected final boolean constant;

	/**
	 * Default constructor
	 */
	public EvalSettings()
	{
		setCPtr(jni_evalSettings());
		constant = false;
	}
	
	/**
	 * Copy constructor
	 * @param old
	 */
	public EvalSettings(EvalSettings old)
	{
		setCPtr(jni_evalSettings(old.getCPtr()));
		constant = false;
	}
	
	/**
	 * Constructor to create constants
	 * @param cptr Pointer to C++ object
	 */
	protected EvalSettings(long cptr)
	{
		setCPtr(cptr);
		constant = true;
	}
	

	/**
	 * @param delaySendingModifieds Toggle for sending modifieds in a single update event after each evaluation.
	 */
	public void setDelaySendingModifieds(boolean delaySendingModifieds)
	{
		if (!constant)
			jni_setDelaySendingModifieds(getCPtr(), delaySendingModifieds);
	}
	
	
	/**
	 * @return current value of delaySendingModifieds
	 */
	public boolean getDelaySendingModifieds()
	{
		return jni_getDelaySendingModifieds(getCPtr());
	}
	
	
	/**
	 * @param prePrintStatement Statement to print before evaluations. 
	 */
	public void setPrePrintStatement(String prePrintStatement)
	{
		if (!constant)
			jni_setPrePrintStatement(getCPtr(), prePrintStatement);
	}
	
	
	/**
	 * @return current value of prePrintStatement
	 */
	public String getPrePrintStatement()
	{
		return jni_getPrePrintStatement(getCPtr());
	}
	
	
	/**
	 * @param postPrintStatement Statement to print after evaluations. 
	 */
	public void setPostPrintStatement(String postPrintStatement)
	{
		if (!constant)
			jni_setPostPrintStatement(getCPtr(), postPrintStatement);
	}
	
	
	/**
	 * @return current value of getPostPrintStatement
	 */
	public String getPostPrintStatement()
	{
		return jni_getPostPrintStatement(getCPtr());
	}

	
	/**
	 * @param alwaysOverwrite Toggle for always overwriting records, regardless of quality, clock values, etc. 
	 */
	public void setAlwaysOverwrite(boolean alwaysOverwrite)
	{
		if (!constant)
			jni_setAlwaysOverwrite(getCPtr(), alwaysOverwrite);
	}
	
	
	/**
	 * @return current value of alwaysOverwrite
	 */
	public boolean getAlwaysOverwrite()
	{
		return jni_getAlwaysOverwrite(getCPtr());
	}
	

	/**
	 * @param treatGlobalsAsLocals Toggle whether updates to global variables are treated as local variables and not marked as modified to the transport. 
	 */
	public void setTreatGlobalsAsLocals(boolean treatGlobalsAsLocals)
	{
		if (!constant)
			jni_setTreatGlobalsAsLocals(getCPtr(), treatGlobalsAsLocals);
	}

	
	/**
	 * @return current value of treatGlobalsAsLocals
	 */
	public boolean getTreatGlobalsAsLocals()
	{
		return jni_getTreatGlobalsAsLocals(getCPtr());
	}

	
	/**
	 * @param defaultClockIncrement Default clock increment. 
	 */
	public void setDefaultClockIncrement(long defaultClockIncrement)
	{
		if (!constant)
			jni_setClockIncrement(getCPtr(), defaultClockIncrement);
	}

	
	/**
	 * @return get the default clock increment
	 */
	public long getDefaultClockIncrement()
	{
		return jni_getClockIncrement(getCPtr());
	}

	/**
	 * Deletes the C instantiation. To prevent memory leaks, this <b>must</b> be called
	 * before an instance of EvalSettings gets garbage collected
	 */
	public void free()
	{
		jni_freeEvalSettings(getCPtr());
	}
	
}
