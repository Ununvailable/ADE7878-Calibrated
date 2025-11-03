#define 	AIGAIN	0x4380	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	current	gain	adjust.			
#define 	AVGAIN	0x4381	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	voltage	gain	adjust.			
#define 	BIGAIN	0x4382	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	current	gain	adjust.			
#define 	BVGAIN	0x4383	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	voltage	gain	adjust.			
#define 	CIGAIN	0x4384	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	current	gain	adjust.			
#define 	CVGAIN	0x4385	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	voltage	gain	adjust.			
#define 	NIGAIN	0x4386	//	R/W	24	32	ZPSE	S	0x000000	Neutral	current	gain	adjust	(ADE7868	and	ADE7878	only).
#define 	AIRMSOS	0x4387	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	current	rms	offset.			
#define 	AVRMSOS	0x4388	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	voltage	rms	offset.			
#define 	BIRMSOS	0x4389	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	current	rms	offset.			
#define 	BVRMSOS	0x438A	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	voltage	rms	offset.			
#define 	CIRMSOS	0x438B	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	current	rms	offset.			
#define 	CVRMSOS	0x438C	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	voltage	rms	offset.			
#define 	NIRMSOS	0x438D	//	R/W	24	32	ZPSE	S	0x000000	Neutral	current	rms	offset	(ADE7868	and	ADE7878	only).
#define 	AVAGAIN	0x438E	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	apparent	power	gain	adjust.		
#define 	BVAGAIN	0x438F	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	apparent	power	gain	adjust.		
#define 	CVAGAIN	0x4390	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	apparent	power	gain	adjust.								
#define 	AWGAIN	0x4391	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	total	active	power	gain	adjust.							
#define 	AWATTOS	0x4392	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	total	active	power	offset	adjust.							
#define 	BWGAIN	0x4393	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	total	active	power	gain	adjust.							
#define 	BWATTOS	0x4394	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	total	active	power	offset	adjust.							
#define 	CWGAIN	0x4395	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	total	active	power	gain	adjust.							
#define 	CWATTOS	0x4396	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	total	active	power	offset	adjust.							
#define 	AVARGAIN	0x4397	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	total	reactive	power	gain	adjust	"(ADE7858,"	"ADE7868,"	and	ADE7878).			
#define 	AVAROS	0x4398	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	total	reactive	power	offset	adjust	"(ADE7858,"	"ADE7868,"	and	ADE7878).			
#define 	BVARGAIN	0x4399	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	total	reactive	power	gain	adjust	"(ADE7858,"	"ADE7868,"	and	ADE7878).			
#define 	BVAROS	0x439A	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	total	reactive	power	offset	adjust	"(ADE7858,"	"ADE7868,"	and	ADE7878).			
#define 	CVARGAIN	0x439B	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	total	reactive	power	gain	adjust	"(ADE7858,"	"ADE7868,"	and	ADE7878).			
#define 	CVAROS	0x439C	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	total	reactive	power	offset	adjust	"(ADE7858,"	"ADE7868,"	and	ADE7878).			
#define 	AFWGAIN	0x439D	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	fundamental	active	power	gain	adjust.	Location	reserved	for	"ADE7854,"	"ADE7858,"	and	ADE7868.
#define 	AFWATTOS	0x439E	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	fundamental	active	power	offset	adjust.	Location	reserved	for	"ADE7854,"	"ADE7858,"	and	ADE7868.
#define 	BFWGAIN	0x439F	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	fundamental	active	power	gain	adjust	(ADE7878	only).					
#define 	BFWATTOS	0x43A0	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	fundamental	active	power	offset	adjust	(ADE7878	only).									
#define 	CFWGAIN	0x43A1	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	fundamental	active	power	gain	adjust.											
#define 	CFWATTOS	0x43A2	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	fundamental	active	power	offset	adjust	(ADE7878	only).									
#define 	AFVARGAIN	0x43A3	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	fundamental	reactive	power	gain	adjust	(ADE7878	only).									
#define 	AFVAROS	0x43A4	//	R/W	24	32	ZPSE	S	0x000000	Phase	A	fundamental	reactive	power	offset	adjust	(ADE7878	only).									
#define 	BFVARGAIN	0x43A5	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	fundamental	reactive	power	gain	adjust	(ADE7878	only).									
#define 	BFVAROS	0x43A6	//	R/W	24	32	ZPSE	S	0x000000	Phase	B	fundamental	reactive	power	offset	adjust	(ADE7878	only).									
#define 	CFVARGAIN	0x43A7	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	fundamental	reactive	power	gain	adjust	(ADE7878	only).									
#define 	CFVAROS	0x43A8	//	R/W	24	32	ZPSE	S	0x000000	Phase	C	fundamental	reactive	power	offset	adjust	(ADE7878	only).									
#define 	VATHR1	0x43A9	//	R/W	24	32	ZP	U	0x000000	Most	significant	24	bits	of	VATHR[47:0]	threshold	used	in	phase	apparent	power	datapath.					
#define 	VATHR0	0x43AA	//	R/W	24	32	ZP	U	0x000000	Less	significant	24	bits	of	VATHR[47:0]	threshold	used	in	phase	apparent	power	datapath.					
#define 	WTHR1	0x43AB	//	R/W	24	32	ZP	U	0x000000	Most	significant	24	bits	of	WTHR[47:0]	threshold	used	in	phase	total/fundamental	active	power	datapath.				
#define 	WTHR0	0x43AC	//	R/W	24	32	ZP	U	0x000000	Less	significant	24	bits	of	WTHR[47:0]	threshold	used	in	phase	total/fundamental	active	power	datapath.				
#define 	VARTHR1	0x43AD	//	R/W	24	32	ZP	U	0x000000	Most	significant	24	bits	of	VARTHR[47:0]	threshold	used	in	phase	total/fundamental	reactive	power	datapath	"(ADE7858,"	"ADE7868,"	and	ADE7878).
#define 	VARTHR0	0x43AE	//	R/W	24	32	ZP	U	0x000000	Less	significant	24	bits	of	VARTHR[47:0]	threshold	used	in	phase	total/fundamental	reactive	power	datapath	"(ADE7858,"	"ADE7868,"	and	ADE7878).
//0x43AF	Reserved	N/A4	N/A4	N/A4	N/A4	0x000000	This	memory	location	should	be	kept	at	0x000000	for	proper	operation.							
#define 	VANOLOAD	0x43B0	//	R/W	24	32	ZPSE	S	0x0000000	No	load	threshold	in	the	apparent	power	datapath.																		
#define 	APNOLOAD	0x43B1	//	R/W	24	32	ZPSE	S	0x0000000	No	load	threshold	in	the	total/fundamental	active	power	datapath.																	
#define 	VARNOLOAD	0x43B2	//	R/W	24	32	ZPSE	S	0x0000000	No	load	threshold	in	the	total/fundamental	reactive	power	datapath.	Location	reserved	for	ADE7854.													
#define 	VLEVEL	0x43B3	//	R/W	24	32	ZPSE	S	0x000000	Register	used	in	the	algorithm	that	computes	the	fundamental	active	and	reactive	powers	(ADE7878	only).											
//0x43B4	Reserved	N/A4	N/A4	N/A4	N/A4	0x000000	This	location	should	not	be	written	for	proper	operation.																	
#define 	DICOEFF	0x43B5	//	R/W	24	32	ZPSE	S	0x0000000	Register	used	in	the	digital	integrator	algorithm.	If	the	integrator	is	turned	"on,"	it	must	be	set	at	0xFF8000.	In	"practice,"	it	is	transmitted	as	0xFFF8000.
#define 	HPFDIS	0x43B6	//	R/W	24	32	ZP	U	0x000000	Disables/enables	the	HPF	in	the	current	datapath	(see	Table	34).																
//0x43B7	Reserved	N/A4	N/A4	N/A4	N/A4	0x000000	This	memory	location	should	be	kept	at	0x000000	for	proper	operation.															
#define 	ISUMLVL	0x43B8	//	R/W	24	32	ZPSE	S	0x000000	Threshold	used	in	comparison	between	the	sum	of	phase	currents	and	the	neutral	current	(ADE7868	and	ADE7878	only).								
//0x43B9	to	0x43BE	Reserved	N/A4	N/A4	N/A4	N/A4	0x000000	These	memory	locations	should	be	kept	at	0x000000	for	proper	operation.													
#define 	ISUM	0x43BF	//	R	28	32	ZP	S	N/A4	Sum	of	"IAWV,"	"IBWV,"	and	ICWV	registers	(ADE7868	and	ADE7878	only).															
#define 	AIRMS	0x43C0	//	R	24	32	ZP	S	N/A4	Phase	A	current	rms	value.																					
#define 	AVRMS	0x43C1	//	R	24	32	ZP	S	N/A4	Phase	A	voltage	rms	value.																					
#define 	BIRMS	0x43C2	//	R	24	32	ZP	S	N/A4	Phase	B	current	rms	value.																					
#define 	BVRMS	0x43C3	//	R	24	32	ZP	S	N/A4	Phase	B	voltage	rms	value.																					
#define 	CIRMS	0x43C4	//	R	24	32	ZP	S	N/A4	Phase	C	current	rms	value.																					
#define 	CVRMS	0x43C5	//	R	24	32	ZP	S	N/A4	Phase	C	voltage	rms	value.										
#define 	NIRMS	0x43C6	//	R	24	32	ZP	S	N/A4	Neutral	current	rms	value	(ADE7868	and	ADE7878	only).							
//0x43C7	to	//	0x43FF	Reserved	N/A4	N/A4	N/A4	N/A4	N/A4	These	memory	locations	should	not	be	written	for	proper	operation.		
//0xE203	Reserved	R/W	16	16	U	0x0000	This	memory	location	should	not	be	written	for	proper	operation.					
#define 	RUN	0xE228	//	R/W	16	16	U	0x0000	Run	register	starts	and	stops	the	DSP.	See	the	Digital	Signal	Processor	section	for	more	details.
#define 	AWATTHR	0xE400	//	R	32	32	S	0x00000000	Phase	A	total	active	energy	accumulation.										
#define 	BWATTHR	0xE401	//	R	32	32	S	0x00000000	Phase	B	total	active	energy	accumulation.										
#define 	CWATTHR	0xE402	//	R	32	32	S	0x00000000	Phase	C	total	active	energy	accumulation.										
#define 	AFWATTHR	0xE403	//	R	32	32	S	0x00000000	Phase	A	fundamental	active	energy	accumulation	(ADE7878	only).								
#define 	BFWATTHR	0xE404	//	R	32	32	S	0x00000000	Phase	B	fundamental	active	energy	accumulation	(ADE7878	only).								
#define 	CFWATTHR	0xE405	//	R	32	32	S	0x00000000	Phase	C	fundamental	active	energy	accumulation	(ADE7878	only).								
#define 	AVARHR	0xE406	//	R	32	32	S	0x00000000	Phase	A	total	reactive	energy	accumulation	"(ADE7858,"	"ADE7868,"	and	ADE7878	only).					
#define 	BVARHR	0xE407	//	R	32	32	S	0x00000000	Phase	B	total	reactive	energy	accumulation	"(ADE7858,"	"ADE7868,"	and	ADE7878	only).					
#define 	CVARHR	0xE408	//	R	32	32	S	0x00000000	Phase	C	total	reactive	energy	accumulation	"(ADE7858,"	"ADE7868,"	and	ADE7878	only).					
#define 	AFVARHR	0xE409	//	R	32	32	S	0x00000000	Phase	A	fundamental	reactive	energy	accumulation	(ADE7878	only).								
#define 	BFVARHR	0xE40A	//	R	32	32	S	0x00000000	Phase	B	fundamental	reactive	energy	accumulation	(ADE7878	only).								
#define 	CFVARHR	0xE40B	//	R	32	32	S	0x00000000	Phase	C	fundamental	reactive	energy	accumulation	(ADE7878	only).									
#define 	AVAHR	0xE40C	//	R	32	32	S	0x00000000	Phase	A	apparent	energy	accumulation.												
#define 	BVAHR	0xE40D	//	R	32	32	S	0x00000000	Phase	B	apparent	energy	accumulation.												
#define 	CVAHR	0xE40E	//	R	32	32	S	0x00000000	Phase	C	apparent	energy	accumulation.												
#define 	IPEAK	0xE500	//	R	32	32	U	N/A	Current	peak	register.	See	Figure	50	and	Table	35	for	details	about	its	composition.			
#define 	VPEAK	0xE501	//	R	32	32	U	N/A	Voltage	peak	register.	See	Figure	50	and	Table	36	for	details	about	its	composition.			
#define 	STATUS0	0xE502	//	R/W	32	32	U	N/A	Interrupt	Status	Register	0	See	Table	37										
#define 	STATUS1	0xE503	//	R/W	32	32	U	N/A	Interrupt	Status	Register	1	See	Table	38										
#define 	AIMAV	0xE504	//	R	20	32	ZP	U	N/A	Phase	A	current	mean	absolute	value	computed	during	PSM0	and	PSM1	modes	(ADE7868	and	ADE7878	only).
#define 	BIMAV	0xE505	//	R	20	32	ZP	U	N/A	Phase	B	current	mean	absolute	value	computed	during	PSM0	and	PSM1	modes	(ADE7868	and	ADE7878	only).
#define 	CIMAV	0xE506	//	R	20	32	ZP	U	N/A	Phase	C	current	mean	absolute	value	computed	during	PSM0	and	PSM1	modes	(ADE7868	and	ADE7878	only).
#define 	OILVL	0xE507	//	R/W	24	32	ZP	U	0xFFFFFF	Overcurrent	threshold.														
#define 	OVLVL	0xE508	//	R/W	24	32	ZP	U	0xFFFFFF	Overvoltage	threshold.														
#define 	SAGLVL	0xE509	//	R/W	24	32	ZP	U	0x000000	Voltage	SAG	level	threshold.												
#define 	MASK0	0xE50A	//	R/W	32	32	U	0x00000000	Interrupt	Enable	Register	0	See	Table	39										
#define 	MASK1	0xE50B	//	R/W	32	32	U	0x00000000	Interrupt	Enable	Register	1	See	Table	40										
#define 	IAWV	0xE50C	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	A	current.							
#define 	IBWV	0xE50D	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	B	current.							
#define 	ICWV	0xE50E	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	C	current.							
#define 	INWV	0xE50F	//	R	24	32	SE	S	N/A	Instantaneous	value	of	neutral	current	(ADE7868	and	ADE7878	only).				
#define 	VAWV	0xE510	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	A	voltage.							
#define 	VBWV	0xE511	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	B	voltage.							
#define 	VCWV	0xE512	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	C	voltage.							
#define 	AWATT	0xE513	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	A	total	active	power.					
#define 	BWATT	0xE514	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	B	total	active	power.					
#define 	CWATT	0xE515	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	C	total	active	power.					
#define 	AVAR	0xE516	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	A	total	reactive	power	"(ADE7858,"	"ADE7868,"	and	ADE7878	only).
#define 	BVAR	0xE517	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	B	total	reactive	power	"(ADE7858,"	"ADE7868,"	and	ADE7878	only).
#define 	CVAR	0xE518	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	C	total	reactive	power	"(ADE7858,"	"ADE7868,"	and	ADE7878	only).
#define 	AVA	0xE519	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	A	apparent	power.						
#define 	BVA	0xE51A	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	B	apparent	power.						
#define 	CVA	0xE51B	//	R	24	32	SE	S	N/A	Instantaneous	value	of	Phase	C	apparent	power.						
#define 	CHECKSUM	0xE51F	//	R	32	32	U	0x33666787	Checksum	verification.	See	the	Checksum	Register	section	for	details.					
#define 	VNOM	0xE520	//	R/W	24	32	ZP	S	0x000000	Nominal	phase	voltage	rms	used	in	the	alternative	computation	of	the	apparent	power
//0xE521	to	0xE52E	Reserved	These	addresses	should	not	be	written	for	proper	operation.							
#define 	PHSTATUS	0xE600	//	R	16	16	U	N/A	Phase	peak	register.	See	Table	41								
#define 	ANGLE0	0xE601	//	R	16	16	U	N/A	Time	Delay	0	See	the	Time	Interval	Between	Phases	section	for	details.		
#define 	ANGLE1	0xE602	//	R	16	16	U	N/A	Time	Delay	1	See	the	Time	Interval	Between	Phases	section	for	details.		
#define 	ANGLE2	0xE603	//	R	16	16	U	N/A	Time	Delay	2	See	the	Time	Interval	Between	Phases	section	for	details.		
//0xE604	to	0xE606	Reserved	These	addresses	should	not	be	written	for	proper	operation.							
#define 	PERIOD	0xE607	//	R	16	16	U	N/A	Network	line	period.											
#define 	PHNOLOAD	0xE608	//	R	16	16	U	N/A	Phase	no	load	register.	See	Table	42							
//0xE609	to	0xE60B	Reserved	These	addresses	should	not	be	written	for	proper	operation.							
#define 	LINECYC	0xE60C	//	R/W	16	16	U	0xFFFF	Line	cycle	accumulation	mode	count.									
#define 	ZXTOUT	0xE60D	//	R/W	16	16	U	0xFFFF	Zero-crossing	timeout	count.											
#define 	COMPMODE	0xE60E	//	R/W	16	16	U	0x01FF	Computation-mode	register.	See	Table	43									
#define 	GAIN	0xE60F	//	R/W	16	16	U	0x0000	PGA	gains	at	ADC	inputs.	See	Table	44						
#define 	CFMODE	0xE610	//	R/W	16	16	U	0x0E88	CFx	configuration	register.	See	Table	45								
#define 	CF1DEN	0xE611	//	R/W	16	16	U	0x0000	CF1	denominator.																
#define 	CF2DEN	0xE612	//	R/W	16	16	U	0x0000	CF2	denominator.																
#define 	CF3DEN	0xE613	//	R/W	16	16	U	0x0000	CF3	denominator.																
#define 	APHCAL	0xE614	//	R/W	10	16	ZP	S	0x0000	Phase	calibration	of	Phase	A.	See	Table	46									
#define 	BPHCAL	0xE615	//	R/W	10	16	ZP	S	0x0000	Phase	calibration	of	Phase	B.	See	Table	46									
#define 	CPHCAL	0xE616	//	R/W	10	16	ZP	S	0x0000	Phase	calibration	of	Phase	C.	See	Table	46									
#define 	PHSIGN	0xE617	//	R	16	16	U	N/A	Power	sign	register.	See	Table	47												
#define 	CONFIG	0xE618	//	R/W	16	16	U	0x0000	ADE7878	configuration	register.	See	Table	48												
#define 	MMODE	0xE700	//	R/W	8	8	U	0x1C	Measurement	mode	register.	See	Table	49												
#define 	ACCMODE	0xE701	//	R/W	8	8	U	0x00	Accumulation	mode	register.	See	Table	50												
#define 	LCYCMODE	0xE702	//	R/W	8	8	U	0x78	Line	accumulation	mode	behavior.	See	Table	52											
#define 	PEAKCYC	0xE703	//	R/W	8	8	U	0x00	Peak	detection	half	line	cycles.													
#define 	SAGCYC	0xE704	//	R/W	8	8	U	0x00	SAG	detection	half	line	cycles.													
#define 	CFCYC	0xE705	//	R/W	8	8	U	0x01	Number	of	CF	pulses	between	two	consecutive	energy	latches.	See	the	Synchronizing	Energy	Registers	with	CFx	Outputs	section.
#define 	HSDC_CFG	0xE706	//	R/W	8	8	U	0x00	HSDC	configuration	register.	See	Table	53												
#define 	VERSION	0xE707	//	R	8	8	U	Version	of	die.																
//0xEBFF	Reserved	8	8	This	address	can	be	used	in	manipulating	the	SS/HSA	pin	when	SPI	is	chosen	as	the	active	port.	See	the	Serial	Interfaces	section	for	details.
#define 	LPOILVL	0xEC00	//	R/W	8	8	U	0x07	Overcurrent	threshold	used	during	PSM2	mode	(ADE7868	and	ADE7878	only).	See	Table	54	in	which	the	register	is	detailed.				
#define 	CONFIG2	0xEC01	//	R/W	8	8	U	0x00	Configuration	register	used	during	PSM1	mode.	See	Table	55														
