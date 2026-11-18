import { useEffect, useRef, useState } from 'react'
import { Check, Copy, ExternalLink, Link, LoaderCircle, X } from 'lucide-react'
import { ModalShell } from '../Modal'
import { cancelCodexLogin, disconnectCodex, getCodexConnection, pollCodexLogin, startCodexLogin,
  getAcpProviders, type AcpProviderStatus, type CodexConnection, type CodexLogin } from '../../services/modelConnections.ts'
import { knownConversationModels, type ConversationOptions } from '../../services/agentConversation.ts'
import styles from './ModelConnections.module.css'

export function ModelConnections({ options, onClose, onChanged }: {
  options?: ConversationOptions | null; onClose(): void; onChanged(): void
}) {
  const [account, setAccount] = useState<CodexConnection | null>(null)
  const [login, setLogin] = useState<CodexLogin | null>(null)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [copied, setCopied] = useState<'code' | 'link' | 'command' | ''>('')
  const [copyError, setCopyError] = useState('')
  const [provider, setProvider] = useState('codex')
  const [acpProviders, setAcpProviders] = useState<AcpProviderStatus[]>([])
  const changed = useRef(onChanged)
  useEffect(() => { changed.current = onChanged }, [onChanged])
  useEffect(() => {
    if (!copied) return
    const timer = setTimeout(() => setCopied(''), 2200)
    return () => clearTimeout(timer)
  }, [copied])
  useEffect(() => {
    let active = true
    getCodexConnection().then(value => { if (active) setAccount(value) })
      .catch(reason => { if (active) setError(String(reason.message ?? reason)) })
    return () => { active = false }
  }, [])
  useEffect(() => {
    let active = true
    getAcpProviders().then(value => { if (active) setAcpProviders(value) }).catch(() => undefined)
    return () => { active = false }
  }, [])
  useEffect(() => {
    if (!login || login.state !== 'pending' || busy || error) return
    let active = true
    const timer = setTimeout(async () => {
      try {
        const next = await pollCodexLogin(login.id)
        if (!active) return
        setLogin(next)
        if (next.state === 'approved') {
          setAccount(await getCodexConnection())
          if (active) changed.current()
        }
      } catch (reason) { if (active) setError(reason instanceof Error ? reason.message : String(reason)) }
    }, login.interval * 1000)
    return () => { active = false; clearTimeout(timer) }
  }, [login, busy, error])

  async function action(operation: () => Promise<void>) {
    if (busy) return
    setBusy(true); setError('')
    try { await operation() }
    catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)) }
    finally { setBusy(false) }
  }
  const pending = login?.state === 'pending'
  async function copy(value: string, kind: 'code' | 'link' | 'command') {
    setCopyError('')
    try { await navigator.clipboard.writeText(value); setCopied(kind) }
    catch { setCopyError('浏览器未允许复制，请选中下方的链接或验证码手动复制。') }
  }
  function close() {
    void action(async () => {
      if (pending) await cancelCodexLogin(login.id)
      onClose()
    })
  }
  const providerOrder = ['moonshot', 'claude', 'openai', 'qwen', 'grok', 'acp']
  const providers = ['codex', ...new Set([...(options?.models.map(model => model.provider).filter((id): id is string => Boolean(id)) ?? []), ...providerOrder])]
  const labels: Record<string, string> = { codex: 'Codex', openai: 'OpenAI', moonshot: 'Kimi', claude: 'Claude', qwen: 'Qwen', grok: 'Grok · xAI', acp: '外部 Agent' }
  const icons: Record<string, string> = { codex: 'openai.svg', openai: 'openai.svg', moonshot: 'moonshot.ico', claude: 'anthropic.png', qwen: 'qwen.png', grok: 'grok.ico' }
  const models = (options?.models ?? knownConversationModels).filter(model => model.provider === provider)
  const ready = models.some(model => model.configured === true)
  const acpCommandEnv: Record<string, string> = {
    'acp:grok': 'LINGTU_ACP_COMMAND', 'acp:kilo': 'LINGTU_KILO_COMMAND',
    'acp:kiro': 'LINGTU_KIRO_COMMAND', 'acp:opencode': 'LINGTU_OPENCODE_COMMAND',
    'acp:registry': 'LINGTU_ACP_REGISTRY_COMMAND',
  }
  return <ModalShell open title="选择模型厂商" onClose={close} className={styles.dialog}
    headerAction={<button type="button" data-modal-cancel disabled={busy} className={styles.close} aria-label="关闭模型连接" onClick={close}><X size={17} /></button>}>
    <div className={styles.tabs} role="tablist" aria-label="模型提供商" onKeyDown={event => {
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return
      event.preventDefault()
      const index = providers.indexOf(provider)
      const next = event.key === 'Home' ? 0 : event.key === 'End' ? providers.length - 1
        : (index + (event.key === 'ArrowRight' ? 1 : -1) + providers.length) % providers.length
      setProvider(providers[next])
      event.currentTarget.querySelectorAll<HTMLButtonElement>('[role="tab"]')[next]?.focus()
    }}>
      {providers.map(id => {
        const providerModels = (options?.models ?? knownConversationModels).filter(model => model.provider === id)
        const providerReady = providerModels.some(model => model.configured === true)
        return <button key={id} id={`connection-tab-${id}`} role="tab" type="button"
        aria-selected={provider === id} aria-controls="model-connection-panel" tabIndex={provider === id ? 0 : -1}
        onClick={() => setProvider(id)}>
        {icons[id] && <img src={`/assets/providers/${icons[id]}`} width={15} height={15} alt="" />}
        <span>{labels[id] ?? id}<small>{providerReady ? '已配置' : '待连接'}</small></span>{id === 'codex' && pending && <i className={styles.tabPending} aria-label="正在等待授权" />}
      </button>
      })}
    </div>
    <section key={provider} id="model-connection-panel" className={styles.panel} role="tabpanel" aria-labelledby={`connection-tab-${provider}`}>
      {provider === 'codex' ? <>
        <div className={styles.summary}>
          <strong>ChatGPT 账号</strong>
          <span className={styles.status} data-ready={account?.connected}>{account?.connected ? '已授权' : pending ? '等待授权' : account ? '未连接' : '读取中…'}</span>
        </div>
        {pending ? <>
          {login.user_code && <div className={styles.code}>
            <span>验证码</span><code>{login.user_code}</code>
            <button type="button" aria-label="复制验证码" title="复制验证码" onClick={() => void copy(login.user_code!, 'code')}>{copied === 'code' ? <Check size={15} /> : <Copy size={15} />}</button>
          </div>}
          {!login.user_code && <div className={styles.code}><span>官方授权</span><a href={login.auth_url || login.verification_url} target="_blank" rel="noreferrer">打开授权页 <ExternalLink size={13} /></a></div>}
          <div className={styles.caption}><span><LoaderCircle size={11} />等待官方授权</span><span>{new Date(login.expires_at * 1000).toLocaleTimeString()} 前有效</span></div>
          <div className={styles.actions}>
            <button className={styles.cancel} disabled={busy} onClick={() => void action(async () => setLogin(await cancelCodexLogin(login.id)))}>取消</button>
            <button type="button" aria-label="复制登录链接" onClick={() => void copy(login.auth_url || login.verification_url, 'link')}>{copied === 'link' ? <Check size={13} /> : <Link size={13} />}{copied === 'link' ? '已复制' : '复制链接'}</button>
            <a className={styles.primary} href={login.auth_url || login.verification_url} target="_blank" rel="noreferrer">前往登录 <ExternalLink size={13} /></a>
          </div>
          <details className={styles.help}><summary>无法打开？</summary><p>复制链接，在常用浏览器中打开，完成官方登录。</p><p>如果官方页面提示需要启用 device code sign-in，请先到 ChatGPT 的 Security Settings 开启 Codex 设备码登录，再重新开始。</p><code>{login.auth_url || login.verification_url}</code></details>
          <span className={styles.srOnly} role="status">{copied === 'link' ? '登录链接已复制' : copied === 'code' ? '验证码已复制' : ''}</span>
          {copyError && <p className={styles.error} role="alert">{copyError}</p>}
        </> : <>
          <p className={styles.description}>{account?.connected ? '模型可用性以实际请求为准' : '使用 ChatGPT 订阅账号连接。'}</p>
          <div className={styles.actions}>
            {account?.connected ? <button disabled={busy} onClick={() => void action(async () => {
              setAccount(await disconnectCodex()); setLogin(null); changed.current()
            })}>断开连接</button> : <button className={styles.primary} disabled={busy} onClick={() => void action(async () => {
              setCopied(''); setCopyError(''); setLogin(await startCodexLogin())
            })}>{busy ? '处理中…' : '登录'}</button>}
          </div>
        </>}
        {login && !pending && <p className={styles.notice} role="status">{{ approved: '授权已保存，可回到对话选择 Codex。', expired: '验证码已过期，请重新登录。', cancelled: '已取消登录。', failed: '授权未完成，请重新登录。' }[login.state as Exclude<CodexLogin['state'], 'pending'>]}</p>}
        {error && <div className={styles.error} role="alert">{error}<button onClick={() => void action(async () => { setAccount(await getCodexConnection()) })}>重试</button></div>}
      </> : <>
        <div className={styles.summary}><strong>{provider === 'acp' ? '本地 ACP Agent' : `${labels[provider] ?? provider} API`}</strong><span className={styles.status} data-ready={ready}>{ready ? '已配置' : '未配置'}</span></div>
        <p className={styles.description}>{ready ? '可在对话中选择模型；可用性以实际请求为准。' : provider === 'acp' ? '安装后由 Agent 自己管理登录，LingTu 只负责连接 ACP。' : '请在 Host 服务端配置 API Key。'}</p>
        {!ready && provider !== 'acp' && <code className={styles.envKey}>{models[0]?.key_env || 'API Key'}</code>}
        {provider === 'acp' && <code className={styles.envKey}>命令由环境变量配置，状态以服务端探测为准</code>}
        <details className={styles.help}><summary>查看模型 · {models.length}</summary><ul>{models.map(model => <li key={model.id}>
          <span>{model.label}</span><small>{model.configured ? '命令可用' : provider === 'acp' ? `设置 ${acpCommandEnv[model.id] ?? 'ACP 命令'}` : '未配置'}</small>
        </li>)}</ul></details>
        {provider === 'acp' && <div className={styles.providerSetup}>
          {models.map(model => {
            const status = acpProviders.find(item => item.id === model.id)
            if (!status) return null
            return <div className={styles.providerRow} key={model.id}>
              <div className={styles.providerMeta}><strong>{status.label}</strong><span data-ready={status.available}>{status.available ? `已发现${status.version ? ` · ${status.version}` : ''}` : '未发现命令'}</span></div>
              <small>{status.available ? `登录由 ${status.label} 自己管理` : `配置 ${status.command_env}`}</small>
              <div className={styles.providerActions}>
                {status.install_url && <a href={status.install_url} target="_blank" rel="noreferrer">安装说明 <ExternalLink size={12} /></a>}
                {status.install_command && status.install_command.startsWith('npm ') && <button type="button" onClick={() => void copy(status.install_command!, 'command')}>{copied === 'command' ? '已复制' : '复制安装命令'}</button>}
                {status.login_command && <button type="button" onClick={() => void copy(status.login_command!, 'command')}>复制登录命令</button>}
              </div>
            </div>
          })}
        </div>}
      </>}
    </section>
  </ModalShell>
}
