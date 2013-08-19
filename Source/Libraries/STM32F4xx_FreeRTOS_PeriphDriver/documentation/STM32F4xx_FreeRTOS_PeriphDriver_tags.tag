<?xml version='1.0' encoding='ISO-8859-1' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>AsyncSerialPort1.h</name>
    <path>/home/julien/Embedded-Dev/STM32-Dev/StdPeriph_Lib/STM32F4xx_StdPeriph_Lib/Source/Libraries/STM32F4xx_FreeRTOS_PeriphDriver/serial/AsyncSerial/</path>
    <filename>_async_serial_port1_8h</filename>
    <includes id="_serial_type_8h" name="SerialType.h" local="yes" imported="no">SerialType.h</includes>
    <class kind="class">AsyncSerialPort1</class>
    <member kind="define">
      <type>#define</type>
      <name>SERIAL_PORT1_BUFFERS_LENGTH</name>
      <anchorfile>_async_serial_port1_8h.html</anchorfile>
      <anchor>a4cbed98ec5d1540f6d4b14119c898898</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SERIAL_PORT1_PREEMP_PRIORITY</name>
      <anchorfile>_async_serial_port1_8h.html</anchorfile>
      <anchor>aa87d1c0333d6d64b53de8fc6579f0eff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SERIAL_PORT1_SUBPRIORITY</name>
      <anchorfile>_async_serial_port1_8h.html</anchorfile>
      <anchor>af64b785c8f7db644f1f0e2b02ce73e2c</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>SerialType.h</name>
    <path>/home/julien/Embedded-Dev/STM32-Dev/StdPeriph_Lib/STM32F4xx_StdPeriph_Lib/Source/Libraries/STM32F4xx_FreeRTOS_PeriphDriver/serial/</path>
    <filename>_serial_type_8h</filename>
    <member kind="enumeration">
      <name>Parity</name>
      <anchorfile>_serial_type_8h.html</anchorfile>
      <anchor>ac47b2bd906d2c843b97ecae6c1eea710</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>StopBits</name>
      <anchorfile>_serial_type_8h.html</anchorfile>
      <anchor>a2e4c31ec0a94db405865b7c241717fbe</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>DataBits</name>
      <anchorfile>_serial_type_8h.html</anchorfile>
      <anchor>a11275f46707b20c44d7b07eb6ca04baf</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>HwFlowCtrl</name>
      <anchorfile>_serial_type_8h.html</anchorfile>
      <anchor>a0ed0ee4a49d7dcf4109ba39a3ca1a8b7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>LinkMode</name>
      <anchorfile>_serial_type_8h.html</anchorfile>
      <anchor>a30da35a35e2d0882c738f6f991905df7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>BaudRate</name>
      <anchorfile>_serial_type_8h.html</anchorfile>
      <anchor>a7654bd82719bfde1c792d7828664dde2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>InterruptSetting</name>
      <anchorfile>_serial_type_8h.html</anchorfile>
      <anchor>ad8005790f3b92862aa09c52f14f58e24</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <name>SerialStatus</name>
      <anchorfile>_serial_type_8h.html</anchorfile>
      <anchor>a71c113451bfafdaf5fcabcd807acd480</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AsyncSerialPort1</name>
    <filename>class_async_serial_port1.html</filename>
    <member kind="function">
      <type>SerialStatus</type>
      <name>portInit</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a25a461dcd724608e7b014d5d14127b0b</anchor>
      <arglist>(Parity iParityConf, StopBits iStopBitConf, DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRateConf, uint8_t iPreempPriority, uint8_t iSubPriority, InterruptSetting iInterruptSetting)</arglist>
    </member>
    <member kind="function">
      <type>SerialStatus</type>
      <name>getCurrentStatus</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>ae5c7825b0eb62d17f937e1db43bd72ab</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setCurrentStatus</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>ad62d4d10e37dc483addf7df23f3cd4dc</anchor>
      <arglist>(SerialStatus iStatus)</arglist>
    </member>
    <member kind="function">
      <type>CQueue &amp;</type>
      <name>getOutStream</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a826613a7951299d92cda2a7dac338636</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>CQueue &amp;</type>
      <name>getInStream</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>aca8bcbacddd2d46ca9460c3d9b232752</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>SerialStatus</type>
      <name>getChar</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>abf7bce398cb06b5368f7a617bcbdf857</anchor>
      <arglist>(const int8_t *oCharacter, portTickType iBlockTime)</arglist>
    </member>
    <member kind="function">
      <type>SerialStatus</type>
      <name>putChar</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a414bdbd17f686e2a31f8109d4492b595</anchor>
      <arglist>(const int8_t iCharacter, portTickType iBlockTime)</arglist>
    </member>
    <member kind="function">
      <type>SerialStatus</type>
      <name>putString</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a1f8e0292a566d5a16e993202a6ea9d1b</anchor>
      <arglist>(const int8_t *const iString, uint32_t *oNbCharSent)</arglist>
    </member>
    <member kind="function">
      <type>portBASE_TYPE</type>
      <name>takeMutex</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a2be5dc99ca1c1edd5153a96f3336e6e8</anchor>
      <arglist>(portTickType iBlockTime)</arglist>
    </member>
    <member kind="function">
      <type>portBASE_TYPE</type>
      <name>giveMutex</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a38c06002c0fb8fbd5b1eb8ea4ef6a69c</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static AsyncSerialPort1 *</type>
      <name>getInstance</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a1c767b7bdd824c5b3fd4cdaa3dfc2081</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>closePort</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a2dd7f83163710334d2dc5be1e53812be</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static AsyncSerialPort1 *</type>
      <name>portHandle</name>
      <anchorfile>class_async_serial_port1.html</anchorfile>
      <anchor>a9661343083970644038cc10ba8e3d0a5</anchor>
      <arglist></arglist>
    </member>
  </compound>
</tagfile>
