'use strict';

var commandLibraryState={query:'',category:'all',showDisabled:false,selected:'help'};

function commandEscape(value){
  return String(value).replace(/[&<>"']/g,function(ch){
    return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch];
  });
}

function commandAvailable(entry){
  return entry.profiles.indexOf(SHELL_CATALOG_META.activeProfile)>=0;
}

function commandRiskBadge(risk){
  var classes={R:'read',W:'write',M:'motion'};
  return'<span class="command-badge '+classes[risk]+'">'+
    commandEscape(risk+' · '+SHELL_CATALOG_META.riskLabels[risk])+'</span>';
}

function commandSearchText(entry){
  return[
    entry.name,entry.title,entry.category,entry.summary,entry.note,
    entry.forms.map(function(form){return form.syntax+' '+form.description}).join(' ')
  ].join(' ').toLowerCase();
}

function commandFilteredEntries(){
  var query=commandLibraryState.query.toLowerCase();
  return SHELL_COMMAND_LIBRARY.filter(function(entry){
    if(!commandLibraryState.showDisabled&&!commandAvailable(entry))return false;
    if(commandLibraryState.category!=='all'&&entry.category!==commandLibraryState.category)return false;
    return !query||commandSearchText(entry).indexOf(query)>=0;
  });
}

function commandRenderCategories(){
  var select=$('commandCategory');
  if(select.options.length)return;
  var categories=[];
  SHELL_COMMAND_LIBRARY.forEach(function(entry){
    if(categories.indexOf(entry.category)<0)categories.push(entry.category);
  });
  select.innerHTML='<option value="all">全部分类</option>'+categories.map(function(category){
    return'<option value="'+commandEscape(category)+'">'+commandEscape(category)+'</option>';
  }).join('');
}

function commandRenderStats(){
  var active=SHELL_COMMAND_LIBRARY.filter(commandAvailable);
  var forms=active.reduce(function(total,entry){return total+entry.forms.length},0);
  var disabled=SHELL_COMMAND_LIBRARY.length-active.length;
  $('commandStats').textContent=SHELL_CATALOG_META.activeProfileLabel+' · '+
    active.length+' 个顶层命令 · '+forms+' 种调用格式 · '+disabled+' 组当前未启用';
}

function commandRenderRows(){
  var entries=commandFilteredEntries();
  var rows=$('commandRows');
  if(entries.length&&entries.every(function(entry){return entry.name!==commandLibraryState.selected})){
    commandLibraryState.selected=entries[0].name;
  }
  $('commandEmpty').hidden=entries.length!==0;
  rows.innerHTML=entries.map(function(entry){
    var available=commandAvailable(entry);
    var badges=commandRiskBadge(entry.risk);
    badges+=available?
      '<span class="command-badge dev">当前可用</span>':
      '<span class="command-badge disabled">当前未启用</span>';
    return'<button class="command-row'+
      (entry.name===commandLibraryState.selected?' active':'')+
      '" type="button" data-command="'+commandEscape(entry.name)+'">'+
      '<span><span class="command-row-title"><span class="command-row-name">'+
      commandEscape(entry.name)+'</span>'+commandEscape(entry.title)+'</span>'+
      '<span class="command-row-summary">'+commandEscape(entry.summary)+'</span></span>'+
      '<span class="command-badges">'+badges+'</span></button>';
  }).join('');

}

function commandSelectedEntry(){
  return SHELL_COMMAND_LIBRARY.find(function(entry){
    return entry.name===commandLibraryState.selected;
  })||null;
}

function commandRenderDetails(){
  var entry=commandSelectedEntry();
  var details=$('commandDetails');
  if(!entry){
    details.innerHTML='<div style="color:#758094;padding:30px 5px">选择一个命令查看详情</div>';
    return;
  }

  var available=commandAvailable(entry);
  var profileBadges=entry.profiles.map(function(profile){
    return'<span class="command-badge dev">'+
      (profile==='development'?'开发配置':'比赛配置')+'</span>';
  }).join('');
  if(!profileBadges)profileBadges='<span class="command-badge disabled">当前两个配置均未启用</span>';

  var warning=available?'':(
    '<div class="command-note warn">该命令处理代码仍在源码中，但当前'+
    commandEscape(SHELL_CATALOG_META.activeProfileLabel)+
    '没有注册它。必须修改功能开关并重新编译、烧录后才能使用。</div>');
  var note=entry.note?'<div class="command-note">'+commandEscape(entry.note)+'</div>':'';
  var forms=entry.forms.map(function(form,index){
    return'<div class="command-form">'+
      '<code>'+commandEscape(form.syntax)+'</code>'+
      '<div class="command-form-desc">'+commandEscape(form.description)+' '+commandRiskBadge(form.risk)+'</div>'+
      '<div class="command-form-actions">'+
      '<button type="button" data-copy-form="'+index+'">复制</button>'+
      '<button type="button" data-fill-form="'+index+'">填入终端</button>'+
      '</div></div>';
  }).join('');

  details.innerHTML=
    '<div class="command-detail-head"><div class="command-detail-head-main">'+
    '<div class="command-detail-name">'+commandEscape(entry.name)+'</div>'+
    '<div class="command-detail-title">'+commandEscape(entry.title)+'</div></div>'+
    '<div class="command-badges">'+commandRiskBadge(entry.risk)+profileBadges+'</div></div>'+
    '<div class="command-detail-summary">'+commandEscape(entry.summary)+'</div>'+
    warning+note+
    '<div class="command-form-title">使用方法 · '+entry.forms.length+' 种调用格式</div>'+
    forms;
}

function commandRenderAll(){
  commandRenderCategories();
  commandRenderStats();
  commandRenderRows();
  commandRenderDetails();
}

function commandCopy(text){
  if(navigator.clipboard&&navigator.clipboard.writeText){
    navigator.clipboard.writeText(text).catch(function(){});
    return;
  }
  var area=document.createElement('textarea');
  area.value=text;
  document.body.appendChild(area);
  area.select();
  document.execCommand('copy');
  document.body.removeChild(area);
}

function commandFormByIndex(index){
  var entry=commandSelectedEntry();
  return(entry&&index>=0&&index<entry.forms.length)?entry.forms[index]:null;
}

function commandFillTerminal(text){
  switchTab('terminal');
  $('termInput').value=text;
  $('termInput').focus();
  $('termInput').setSelectionRange(text.length,text.length);
}

function CommandLibrary_OnShow(){
  commandRenderAll();
  $('commandSearch').focus();
}

$('commandSearch').addEventListener('input',function(){
  commandLibraryState.query=this.value.trim();
  commandRenderRows();
  commandRenderDetails();
});

$('commandCategory').addEventListener('change',function(){
  commandLibraryState.category=this.value;
  commandRenderRows();
  commandRenderDetails();
});

$('commandShowDisabled').addEventListener('change',function(){
  commandLibraryState.showDisabled=this.checked;
  commandRenderRows();
  commandRenderDetails();
});

$('commandRows').addEventListener('click',function(event){
  var row=event.target.closest('[data-command]');
  if(!row)return;
  commandLibraryState.selected=row.dataset.command;
  commandRenderRows();
  commandRenderDetails();
});

$('commandDetails').addEventListener('click',function(event){
  var copy=event.target.closest('[data-copy-form]');
  var fill=event.target.closest('[data-fill-form]');
  var target=copy||fill;
  if(!target)return;
  var form=commandFormByIndex(Number(target.dataset.copyForm||target.dataset.fillForm));
  if(!form)return;
  if(copy)commandCopy(form.syntax);
  else commandFillTerminal(form.syntax);
});

/* Keep terminal completion aligned with the current development profile. */
TERM_COMMANDS=SHELL_COMMAND_LIBRARY.filter(commandAvailable).map(function(entry){return entry.name});
commandRenderAll();
