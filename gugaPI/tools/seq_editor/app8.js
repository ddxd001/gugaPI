'use strict';

var helpState={
  category:'all',
  query:'',
  activeId:'quick-start',
  entries:HelpCatalog.buildEntries(SC.ACTIONS)
};

function helpEsc(value){
  return String(value==null?'':value).replace(/[&<>"']/g,function(char){
    return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',
      "'":'&#39;'}[char];
  });
}

function helpRiskLabel(risk){
  return{
    safe:'说明',
    warning:'注意',
    danger:'高风险',
    motion:'可能运动',
    output:'输出状态'
  }[risk]||'说明';
}

function helpCategoryEntries(category){
  return HelpCatalog.search(helpState.entries,helpState.query,category);
}

function helpRenderCategories(){
  var matches=HelpCatalog.search(helpState.entries,helpState.query,'all');
  var rows=[{
    id:'all',label:'全部内容',description:'查看所有帮助文章',
    count:matches.length
  }].concat(HelpCatalog.CATEGORIES.map(function(category){
    return{
      id:category.id,
      label:category.label,
      description:category.description,
      count:matches.filter(function(entry){
        return entry.category===category.id;
      }).length
    };
  }));
  $('helpCategories').innerHTML=rows.map(function(row){
    return'<button class="help-category'+
      (row.id===helpState.category?' active':'')+
      '" type="button" data-help-category="'+helpEsc(row.id)+'">'+
      '<strong>'+helpEsc(row.label)+'<span>'+row.count+'</span></strong>'+
      '<small>'+helpEsc(row.description)+'</small></button>';
  }).join('');
  $('helpCategories').querySelectorAll('[data-help-category]').forEach(
    function(button){
      button.onclick=function(){
        helpState.category=button.getAttribute('data-help-category');
        helpRender();
      };
    });
}

function helpEntryColor(entry){
  if(entry.kind==='action'&&entry.action)return entry.action.color;
  return{
    quick:'#4da3d9',safety:'#e06767',sequence:'#967bdc',
    dashboard:'#4da3d9',parameters:'#d29b45',terminal:'#7cbf8f',
    troubleshooting:'#dc7181'
  }[entry.category]||'#697382';
}

function helpRenderResults(entries){
  var results=$('helpResults');
  $('helpEmpty').hidden=entries.length!==0;
  if(!entries.length){
    results.innerHTML='';
    $('helpContent').innerHTML='<div class="help-detail"><h1>没有匹配内容</h1>'+
      '<p class="help-summary">换一个关键词，或点击“清除搜索”查看全部帮助。</p></div>';
    $('helpStats').textContent='0 条';
    helpState.activeId=null;
    return;
  }
  if(!entries.some(function(entry){return entry.id===helpState.activeId})){
    helpState.activeId=entries[0].id;
  }
  results.innerHTML=entries.map(function(entry){
    return'<button class="help-result'+
      (entry.id===helpState.activeId?' active':'')+
      '" type="button" data-help-entry="'+helpEsc(entry.id)+'">'+
      '<span class="help-result-head"><i class="help-result-dot" style="background:'+
      helpEsc(helpEntryColor(entry))+'"></i><span class="help-result-title">'+
      helpEsc(entry.title)+'</span><span class="help-result-type">'+
      helpEsc(entry.kind==='action'?entry.actionType:entry.category)+
      '</span></span><p>'+helpEsc(entry.summary)+'</p></button>';
  }).join('');
  results.querySelectorAll('[data-help-entry]').forEach(function(button){
    button.onclick=function(){
      helpState.activeId=button.getAttribute('data-help-entry');
      helpRender();
    };
  });
  $('helpStats').textContent=entries.length+' 条';
}

function helpRenderSection(section){
  var html='<section class="help-section"><h2>'+helpEsc(section.title)+'</h2>';
  if(section.body){
    html+=section.body.map(function(paragraph){
      return'<p>'+helpEsc(paragraph)+'</p>';
    }).join('');
  }
  if(section.steps){
    html+='<ol>'+section.steps.map(function(step){
      return'<li>'+helpEsc(step)+'</li>';
    }).join('')+'</ol>';
  }
  return html+'</section>';
}

function helpRenderArticle(entry){
  var category=HelpCatalog.category(entry.category);
  return'<article class="help-detail"><div class="help-detail-kicker">'+
    helpEsc(category?category.label:'使用帮助')+'</div><h1>'+
    helpEsc(entry.title)+'</h1><p class="help-summary">'+
    helpEsc(entry.summary)+'</p><div class="help-badges"><span class="help-badge '+
    helpEsc(entry.risk)+'">'+helpEsc(helpRiskLabel(entry.risk))+
    '</span></div>'+entry.sections.map(helpRenderSection).join('')+'</article>';
}

function helpValue(value){
  if(typeof value==='string')return value;
  return JSON.stringify(value);
}

function helpDemoPortLabel(type,port){
  if(type==='system_start')return'开始';
  if(type==='condition')return port==='success'?'条件成立':'不成立/超时';
  if(type==='loop')return port==='success'?'执行循环体':'循环完成';
  if(type==='road_nav')return port==='success'?'通过并重捕线':'路线不可用/失败';
  return port==='success'?'完成':'失败/超时';
}

function helpDemoNode(node){
  var system=node.type==='system_start';
  var action=system?null:SC.ACTIONS[node.type];
  if(!system&&!action)return'';
  var name=node.label||(system?'开始':action.name);
  var color=system?'#5caf7d':action.color;
  var ports=system?['success']:SC.ports(node.type);
  return'<div class="help-seq-node'+(system?' system':'')+
    '" style="--node-color:'+helpEsc(color)+'">'+
    (system?'':'<i class="help-seq-input"></i>')+
    '<span class="help-seq-stripe"></span>'+
    '<strong>'+helpEsc(name)+'</strong>'+
    (system?'<small>流程入口</small>':
      '<code>'+helpEsc(node.type+' · op '+action.op)+'</code>'+
      '<small>'+helpEsc(node.summary)+'</small>')+
    ports.map(function(port){
      return'<span class="help-seq-port '+port+
        '"><em>'+helpEsc(helpDemoPortLabel(node.type,port))+
        '</em><i></i></span>';
    }).join('')+'</div>';
}

function helpDemoLink(source,port){
  var loopDone=source.type==='loop'&&port==='failure';
  var failed=port==='failure'&&!loopDone;
  var y=source.type==='system_start'?42:(port==='failure'?62:30);
  var color=failed?'#e06c75':'#79bf8a';
  return'<div class="help-seq-link '+(failed?'failure':'success')+'">'+
    '<span style="top:'+(y-18)+'px">'+
    helpEsc(helpDemoPortLabel(source.type,port))+'</span>'+
    '<svg viewBox="0 0 84 84" aria-hidden="true">'+
    '<path d="M0 '+y+' C 28 '+y+', 56 42, 78 42" fill="none" stroke="'+
    color+'" stroke-width="2.2"'+
    (failed?' stroke-dasharray="7 5"':'')+'/>'+
    '<path d="M76 36 L84 42 L76 48 Z" fill="'+color+'"/></svg></div>';
}

function helpRenderFlow(example){
  return'<section class="help-section"><h2>使用案例</h2><div class="help-example">'+
    '<div class="help-example-title">'+helpEsc(example.title)+'</div>'+
    example.paths.map(function(path){
      var diagram='';
      path.nodes.forEach(function(node,index){
        diagram+=helpDemoNode(node);
        if(index<path.nodes.length-1){
          diagram+=helpDemoLink(node,path.ports[index]||'success');
        }
      });
      return'<div class="help-seq-example-path"><div class="help-seq-path-label">'+
        helpEsc(path.label)+'</div><div class="help-seq-path-scroll">'+
        '<div class="help-seq-path">'+diagram+'</div></div>'+
        (path.note?'<p class="help-seq-note">'+helpEsc(path.note)+'</p>':'')+
        '</div>';
    }).join('')+'</div></section>';
}

function helpRenderAction(entry){
  var action=entry.action,guide=entry.guide;
  var defaults=Object.keys(action.defaults).map(function(key){
    return'<div class="help-default"><span>'+helpEsc(key)+
      '</span><code>'+helpEsc(helpValue(action.defaults[key]))+'</code></div>';
  }).join('');
  return'<article class="help-detail"><div class="help-detail-kicker">'+
    '<i style="background:'+helpEsc(action.color)+'"></i>'+
    helpEsc(action.group)+'</div><h1>'+helpEsc(action.name)+'</h1>'+
    '<p class="help-summary">'+helpEsc(guide.purpose)+'</p>'+
    '<div class="help-badges"><span class="help-badge '+helpEsc(guide.risk)+'">'+
    helpEsc(helpRiskLabel(guide.risk))+'</span><span class="help-badge safe">'+
    '类型 '+helpEsc(entry.actionType)+'</span><span class="help-badge safe">'+
    'ActionOp '+helpEsc(action.op)+'</span></div>'+
    '<section class="help-section"><h2>参数说明</h2><ul>'+
    guide.parameters.map(function(parameter){
      return'<li>'+helpEsc(parameter)+'</li>';
    }).join('')+'</ul>'+(defaults?'<div class="help-defaults">'+defaults+
      '</div>':'')+'</section>'+
    '<section class="help-section"><h2>出口行为</h2><div class="help-port-grid">'+
    '<div class="help-port"><strong>绿色 · 正常出口</strong><p>'+
    helpEsc(guide.success)+'</p></div><div class="help-port failure">'+
    '<strong>红色 · 失败/超时出口</strong><p>'+helpEsc(guide.failure)+
    '</p></div></div></section>'+
    helpRenderFlow(guide.example)+
    '<section class="help-section"><h2>使用注意</h2>'+
    guide.tips.map(function(tip){
      return'<div class="help-tip">'+helpEsc(tip)+'</div>';
    }).join('')+'</section></article>';
}

function helpRenderDetail(entry){
  $('helpContent').innerHTML=entry.kind==='action'?
    helpRenderAction(entry):helpRenderArticle(entry);
  $('helpContent').scrollTop=0;
}

function helpRender(){
  helpRenderCategories();
  var entries=helpCategoryEntries(helpState.category);
  helpRenderResults(entries);
  var active=entries.find(function(entry){
    return entry.id===helpState.activeId;
  });
  if(active)helpRenderDetail(active);
}

function HelpPage_Open(entryId){
  var entry=helpState.entries.find(function(item){return item.id===entryId});
  if(!entry)return false;
  helpState.query='';
  helpState.category=entry.category;
  helpState.activeId=entry.id;
  $('helpSearch').value='';
  switchTab('help');
  helpRender();
  return true;
}

function HelpPage_OnShow(){
  helpRender();
}

$('helpSearch').addEventListener('input',function(){
  helpState.query=this.value;
  if(helpState.query.trim())helpState.category='all';
  helpRender();
});
$('btnHelpClear').onclick=function(){
  helpState.query='';
  helpState.category='all';
  $('helpSearch').value='';
  helpRender();
  $('helpSearch').focus();
};

helpRender();
