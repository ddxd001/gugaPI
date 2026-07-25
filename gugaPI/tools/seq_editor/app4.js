// ===== Terminal =====
var termHistory=[];
var termHistIdx=0;
var CMD_LIST=['version','reset','sched','txstat','led','buzzer','button','fram','param','ina219','oled','gy931','imu','gray','lora','motor','chassis','heading','run','lf','comp','telem','seq','i2c','adc','pwm'];

// RX callback - display in terminal
onRxCb=function(data,type){
  var td=$('termDisplay');
  var span=document.createElement('span');
  span.className=type;
  span.textContent=data;
  td.appendChild(span);
  td.scrollTop=td.scrollHeight;
};

// Tab switching
function switchTab(name){
  var ev=$('editorView'),tv=$('termView'),te=$('tabEditor'),tt=$('tabTerminal');
  if(name==='terminal'){ev.style.display='none';tv.style.display='flex';te.classList.remove('active');tt.classList.add('active');$('termInput').focus()}
  else{ev.style.display='flex';tv.style.display='none';te.classList.add('active');tt.classList.remove('active')}
}

// Terminal input handler
var ti=$('termInput');
ti.addEventListener('keydown',function(e){
  if(e.key==='Enter'){
    e.preventDefault();
    var cmd=ti.value.trim();
    if(!cmd)return;
    // Add to history
    if(termHistory.length===0||termHistory[termHistory.length-1]!==cmd){
      termHistory.push(cmd);
      if(termHistory.length>100)termHistory.shift();
    }
    termHistIdx=termHistory.length;
    ti.value='';
    // Send command
    if(simMode){
      send(cmd).then(function(){});
    }else if(writer){
      send(cmd).then(function(){});
    }else{
      var td=$('termDisplay');
      var span=document.createElement('span');
      span.className='tx';
      span.textContent='> '+cmd+'\n';
      td.appendChild(span);
      td.scrollTop=td.scrollHeight;
    }
  }else if(e.key==='ArrowUp'){
    e.preventDefault();
    if(termHistIdx>0){termHistIdx--;ti.value=termHistory[termHistIdx]||'';ti.setSelectionRange(ti.value.length,ti.value.length)}
  }else if(e.key==='ArrowDown'){
    e.preventDefault();
    if(termHistIdx<termHistory.length-1){termHistIdx++;ti.value=termHistory[termHistIdx]||''}
    else{termHistIdx=termHistory.length;ti.value=''}
    ti.setSelectionRange(ti.value.length,ti.value.length);
  }else if(e.key==='Tab'){
    e.preventDefault();
    var val=ti.value.trim();
    var parts=val.split(/\s+/);
    var word=parts[parts.length-1];
    var matches=CMD_LIST.filter(function(c){return c.startsWith(word)});
    if(matches.length===1){
      parts[parts.length-1]=matches[0];
      ti.value=parts.join(' ')+' ';
    }else if(matches.length>1){
      // Show suggestions in terminal
      var td=$('termDisplay');
      var span=document.createElement('span');
      span.className='hint';
      span.textContent=matches.join('  ')+'\n';
      td.appendChild(span);
      td.scrollTop=td.scrollHeight;
    }
  }
});
