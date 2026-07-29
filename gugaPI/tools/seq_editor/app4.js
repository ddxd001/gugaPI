'use strict';

// Shared Web Serial terminal for the gugaPI UART6 debug shell.
var termHistory=[];
var termHistoryIndex=0;
var TERM_MAX_CHARS=200000;
var TERM_COMMANDS=[
  'help','version','reset','sched','txstat','led','buzzer','button','fram',
  'param','oled','imu','gray','lora','motor','chassis','heading','run','lf',
  'road','dm','comp','estop','telem','seq','i2c','can','jyme02','linesensor','irsensor'
];

function Terminal_ShouldShowTelemetry(){
  return $('termShowTelemetry').checked;
}

function terminalAppend(data,type){
  var display=$('termDisplay');
  var span=document.createElement('span');
  span.className=type||'rx';
  span.textContent=data;
  display.appendChild(span);

  while(display.textContent.length>TERM_MAX_CHARS&&display.firstChild){
    display.removeChild(display.firstChild);
  }
  display.scrollTop=display.scrollHeight;
}

function terminalSetConnected(connected){
  $('termInput').disabled=!connected;
  $('btnTermSend').disabled=!connected;
  $('btnTermHelp').disabled=!connected;
  $('termInput').placeholder=connected?'输入命令后回车发送...':'请先连接 gugaPI 串口';
  if(connected){
    terminalAppend(simMode?'[模拟串口已连接]\n':'[gugaPI 串口已连接：115200 8N1]\n','hint');
    if(!$('termView').hidden)$('termInput').focus();
  }else{
    terminalAppend('[串口已断开]\n','hint');
  }
}

onSerialData=terminalAppend;
onSerialStateChange=terminalSetConnected;

var currentView='editor';
function switchTab(name){
  currentView=name;
  var editor=name==='editor';
  var dashboard=name==='dashboard';
  var lineSensor=name==='linesensor';
  var terminal=name==='terminal';
  var parameters=name==='parameters';
  var commands=name==='commands';
  var help=name==='help';
  $('editorView').hidden=!editor;
  $('dashboardView').hidden=!dashboard;
  $('lineSensorView').hidden=!lineSensor;
  $('termView').hidden=!terminal;
  $('paramView').hidden=!parameters;
  $('commandView').hidden=!commands;
  $('helpView').hidden=!help;
  $('log').hidden=!editor;
  $('tabEditor').classList.toggle('active',editor);
  $('tabDashboard').classList.toggle('active',dashboard);
  $('tabLineSensor').classList.toggle('active',lineSensor);
  $('tabTerminal').classList.toggle('active',terminal);
  $('tabParameters').classList.toggle('active',parameters);
  $('tabCommands').classList.toggle('active',commands);
  $('tabHelp').classList.toggle('active',help);
  if(terminal&&!$('termInput').disabled)$('termInput').focus();
  if(dashboard&&typeof Dashboard_OnShow==='function')Dashboard_OnShow();
  if(!dashboard&&typeof Dashboard_OnHide==='function')Dashboard_OnHide();
  if(lineSensor&&typeof LineSensorPage_OnShow==='function')LineSensorPage_OnShow();
  if(!lineSensor&&typeof LineSensorPage_OnHide==='function')LineSensorPage_OnHide();
  if(parameters&&typeof ParamPage_OnShow==='function')ParamPage_OnShow();
  if(!parameters&&typeof ParamPage_OnHide==='function')ParamPage_OnHide();
  if(commands&&typeof CommandLibrary_OnShow==='function')CommandLibrary_OnShow();
  if(help&&typeof HelpPage_OnShow==='function')HelpPage_OnShow();
}

async function terminalSend(){
  var input=$('termInput');
  var command=input.value.trim();
  if(!command||input.disabled)return;

  if(termHistory.length===0||termHistory[termHistory.length-1]!==command){
    termHistory.push(command);
    if(termHistory.length>100)termHistory.shift();
  }
  termHistoryIndex=termHistory.length;
  input.value='';
  try{
    await send(command);
  }catch(error){
    terminalAppend('[发送失败] '+error.message+'\n','error');
  }
}

function terminalComplete(){
  var input=$('termInput');
  var beforeCursor=input.value.slice(0,input.selectionStart);
  if(beforeCursor.trim().includes(' '))return;
  var prefix=beforeCursor.trim();
  var matches=TERM_COMMANDS.filter(function(command){return command.startsWith(prefix)});
  if(matches.length===1){
    input.value=matches[0]+' ';
    input.setSelectionRange(input.value.length,input.value.length);
  }else if(matches.length>1){
    terminalAppend(matches.join('  ')+'\n','hint');
  }
}

$('tabEditor').addEventListener('click',function(){switchTab('editor')});
$('tabDashboard').addEventListener('click',function(){switchTab('dashboard')});
$('tabLineSensor').addEventListener('click',function(){switchTab('linesensor')});
$('tabTerminal').addEventListener('click',function(){switchTab('terminal')});
$('tabParameters').addEventListener('click',function(){switchTab('parameters')});
$('tabCommands').addEventListener('click',function(){switchTab('commands')});
$('tabHelp').addEventListener('click',function(){switchTab('help')});
$('btnTermClear').addEventListener('click',function(){$('termDisplay').textContent=''});
$('btnTermSend').addEventListener('click',terminalSend);
$('btnTermHelp').addEventListener('click',function(){
  $('termInput').value='help';
  terminalSend();
});
$('termInput').addEventListener('keydown',function(event){
  if(event.key==='Enter'){
    event.preventDefault();
    terminalSend();
  }else if(event.key==='ArrowUp'){
    event.preventDefault();
    if(termHistoryIndex>0)termHistoryIndex--;
    this.value=termHistory[termHistoryIndex]||'';
    this.setSelectionRange(this.value.length,this.value.length);
  }else if(event.key==='ArrowDown'){
    event.preventDefault();
    if(termHistoryIndex<termHistory.length-1){
      termHistoryIndex++;
      this.value=termHistory[termHistoryIndex]||'';
    }else{
      termHistoryIndex=termHistory.length;
      this.value='';
    }
    this.setSelectionRange(this.value.length,this.value.length);
  }else if(event.key==='Tab'){
    event.preventDefault();
    terminalComplete();
  }else if(event.ctrlKey&&event.key.toLowerCase()==='l'){
    event.preventDefault();
    $('termDisplay').textContent='';
  }
});

if(!('serial' in navigator)){
  terminalAppend('[当前浏览器不支持 Web Serial，请使用 Chrome 或 Edge。]\n','error');
}
