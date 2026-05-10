from dashscope import Application
import dashscope
from openai import OpenAI
import os
import piper
import wave
from http import HTTPStatus
from dashscope.audio.asr import Recognition
from funasr import AutoModel
from dashscope.audio.tts_v2 import *
from dashscope.audio.asr import *
from ament_index_python.packages import get_package_share_directory
from utils.dify_client2 import CompletionClient, ChatClient
from utils.promot import get_prompt
import yaml
import base64
import requests
import json
import netifaces
from urllib.request import urlopen
from urllib.request import Request
from urllib.error import URLError
from urllib.parse import urlencode
from urllib.parse import quote_plus

import websocket
import datetime
import hashlib
import base64
import hmac
import json
from urllib.parse import urlencode
import time
import ssl
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread
from subprocess import Popen

xufei = ""
Ws_Param = ""

STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识
record_speech_file = os.path.join(
    get_package_share_directory("largemodel"), "resources_file", "user_speech.wav"
)


class Ws_Param(object):
    # 初始化
    def __init__(self, APPID, APIKey, APISecret, AudioFile):

        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.AudioFile = AudioFile

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {
            "domain": "iat",
            "language": "en_us",
            "accent": "mandarin",
            "vinfo": 1,
            "vad_eos": 10000,
        }

    # 生成url
    def create_url(self):
        url = "wss://ws-api.xfyun.cn/v2/iat"
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/iat " + "HTTP/1.1"
        # 进行hmac-sha256进行加密
        signature_sha = hmac.new(
            self.APISecret.encode("utf-8"),
            signature_origin.encode("utf-8"),
            digestmod=hashlib.sha256,
        ).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding="utf-8")

        authorization_origin = (
            'api_key="%s", algorithm="%s", headers="%s", signature="%s"'
            % (self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        )
        authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode(
            encoding="utf-8"
        )
        # 将请求的鉴权参数组合为字典
        v = {"authorization": authorization, "date": date, "host": "ws-api.xfyun.cn"}
        # 拼接鉴权参数，生成url
        url = url + "?" + urlencode(v)
        return url


# 收到websocket消息的处理
def on_message(ws, message):

    try:
        code = json.loads(message)["code"]
        sid = json.loads(message)["sid"]
        if code != 0:
            errMsg = json.loads(message)["message"]
            # print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            data = json.loads(message)["data"]["result"]["ws"]

            result = ""
            for i in data:
                for w in i["cw"]:
                    result += w["w"]

            global xufei
            xufei += result

    except Exception as e:
        print("receive msg,but parse exception:", e)


# 收到websocket错误的处理
def on_error(ws, error):
    print("### error:", error)


# 收到websocket关闭的处理
def on_close(ws, a, b):
    # print("###speak iat closed ###")
    return


# 收到websocket连接建立的处理
def on_open(ws):
    def run(*args):
        frameSize = 8000  # 每一帧的音频大小
        intervel = 0.04  # 发送音频间隔(单位:s)
        status = (
            STATUS_FIRST_FRAME  # 音频的状态信息，标识音频是第一帧，还是中间帧、最后一帧
        )

        with open(wsParam.AudioFile, "rb") as fp:
            while True:
                buf = fp.read(frameSize)
                # 文件结束
                if not buf:
                    status = STATUS_LAST_FRAME
                # 第一帧处理
                # 发送第一帧音频，带business 参数
                # appid 必须带上，只需第一帧发送
                if status == STATUS_FIRST_FRAME:

                    d = {
                        "common": wsParam.CommonArgs,
                        "business": wsParam.BusinessArgs,
                        "data": {
                            "status": 0,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        },
                    }
                    d = json.dumps(d)
                    ws.send(d)
                    status = STATUS_CONTINUE_FRAME
                # 中间帧处理
                elif status == STATUS_CONTINUE_FRAME:
                    d = {
                        "data": {
                            "status": 1,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        }
                    }
                    ws.send(json.dumps(d))
                # 最后一帧处理
                elif status == STATUS_LAST_FRAME:
                    d = {
                        "data": {
                            "status": 2,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        }
                    }
                    ws.send(json.dumps(d))
                    time.sleep(1)
                    break
                # 模拟音频采样间隔
                time.sleep(intervel)
        ws.close()

    thread.start_new_thread(run, ())


wsParam = ""
XUNFEI_TTS_FILE = os.path.join(
    get_package_share_directory("largemodel"), "resources_file", "XUNFEI_TTS.mp3"
)


class Ws_Param_1(object):
    # 初始化 initialization
    def __init__(self, APPID, APIKey, APISecret, Text):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.Text = Text

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {
            "aue": "lame",
            "sfl": 1,
            "auf": "audio/L16;rate=16000",
            "vcn": "x4_xiaoyan",
            "tte": "utf8",
            "speed": 50,
            "pitch": 50,
        }
        self.Data = {
            "status": 2,
            "text": str(base64.b64encode(self.Text.encode("utf-8")), "UTF8"),
        }
        # 使用小语种须使用以下方式，此处的unicode指的是 utf16小端的编码方式，即"UTF-16LE"”
        # self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-16')), "UTF8")}

    # 生成url Generate URL
    def create_url_1(self):
        url = "wss://tts-api.xfyun.cn/v2/tts"
        # 生成RFC1123格式的时间戳 Generate timestamp in RFC1123 format
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串 Splicing strings
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/tts " + "HTTP/1.1"
        # 进行hmac-sha256进行加密 Encrypt hmac-sha256
        signature_sha = hmac.new(
            self.APISecret.encode("utf-8"),
            signature_origin.encode("utf-8"),
            digestmod=hashlib.sha256,
        ).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding="utf-8")

        authorization_origin = (
            'api_key="%s", algorithm="%s", headers="%s", signature="%s"'
            % (self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        )
        authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode(
            encoding="utf-8"
        )
        # 将请求的鉴权参数组合为字典 Combine the requested authentication parameters into a dictionary
        v = {"authorization": authorization, "date": date, "host": "ws-api.xfyun.cn"}
        # 拼接鉴权参数，生成url Splicing authentication parameters and generating URLs
        url = url + "?" + urlencode(v)
        return url


def on_message_1(ws, message):
    try:
        message = json.loads(message)
        code = message["code"]
        sid = message["sid"]
        audio = message["data"]["audio"]
        audio = base64.b64decode(audio)
        status = message["data"]["status"]
        # print(message)
        if status == 2:
            # print("ws is closed")
            ws.close()
        if code != 0:
            errMsg = message["message"]
            print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            with open(XUNFEI_TTS_FILE, "ab") as f:
                f.write(audio)
    except Exception as e:
        print("receive msg,but parse exception:", e)


# 收到websocket错误的处理 Handling of websocket errors received
def on_error_1(ws, error):
    print("### error:", error)


def on_close_1(ws, close_status_code, close_msg):
    return


# 收到websocket连接建立的处理 Received processing for establishing websocket connection
def on_open_1(ws):
    def run(*args):
        d = {
            "common": wsParam.CommonArgs,
            "business": wsParam.BusinessArgs,
            "data": wsParam.Data,
        }
        d = json.dumps(d)
        # print("------>开始发送文本数据")
        ws.send(d)
        if os.path.exists(XUNFEI_TTS_FILE):
            os.remove(XUNFEI_TTS_FILE)

    thread.start_new_thread(run, ())


class model_interface:
    def __init__(self):
        self.init_config_param()
        dashscope.api_key = self.tongyi_api_key

    def init_config_param(self):
        self.pkg_path = get_package_share_directory("largemodel")
        config_param_file = os.path.join(
            self.pkg_path, "config", "large_model_interface.yaml"
        )
        with open(config_param_file, "r") as file:
            config_param = yaml.safe_load(file)
        self.tongyi_api_key = config_param.get("tongyi_api_key")
        self.tongyi_base_url = config_param.get("tongyi_base_url")
        self.tongyi_app_id = config_param.get("tongyi_app_id")
        self.oline_asr_model = config_param.get("oline_asr_model")
        self.zh_tts_model = config_param.get("zh_tts_model")
        self.zh_tts_json = config_param.get("zh_tts_json")
        self.en_tts_model = config_param.get("en_tts_model")
        self.en_tts_json = config_param.get("en_tts_json")
        self.multimodel = config_param.get("multimodel")
        self.ANYTHINGLLM_BASE_URL = config_param.get("ANYTHINGLLM_BASE_URL")
        self.API_KEY = config_param.get("API_KEY")
        self.WORKSPACE_SLUG = config_param.get("WORKSPACE_SLUG")
        self.oline_asr_sample_rate = config_param.get("oline_asr_sample_rate")
        self.oline_tts_model = config_param.get("oline_tts_model")
        self.voice_tone = config_param.get("voice_tone")
        self.local_asr_model = config_param.get("local_asr_model")
        self.tts_supplier = config_param.get("tts_supplier")
        self.baidu_API_KEY = config_param.get("baidu_API_KEY")
        self.baidu_SECRET_KEY = config_param.get("baidu_SECRET_KEY")
        self.CUID = config_param.get("CUID")
        self.PER = config_param.get("PER")
        self.SPD = config_param.get("SPD")
        self.PIT = config_param.get("PIT")
        self.VOL = config_param.get("VOL")
        self.decision_AI_api_key = config_param.get("decision_AI_api_key")
        self.execution_AI_api_key = config_param.get("execution_AI_api_key")
        self.network_adapter = config_param.get("network_adapter")

        self.decision_id = None  # dify决策层id
        self.execution_id = None  # dify执行层id
        self.international_mode = False  # 是否启用国际模式，默认为国内模式

    def init_dify_client(self):
        self.international_mode = True
        self.user = "yahboom"
        self.decision_client = ChatClient(
            self.decision_AI_api_key, base_url="http://localhost/v1"
        )
        self.execution_client = ChatClient(
            self.execution_AI_api_key, base_url="http://localhost/v1"
        )
        if self.decision_client is not None:
            return True
        else:
            return False

    def init_Multimodal(self):
        self.multimodal_client = OpenAI(
            api_key=self.tongyi_api_key, base_url=self.tongyi_base_url
        )
        self.init_Multimodal_history(get_prompt())

    def init_Multimodal_history(self, system_prompt):
        self.Multimodalmessages = []
        self.Multimodalmessages.append(
            {"role": "user", "content": [{"type": "text", "text": system_prompt}]}
        )
        self.Multimodalmessages.append(
            {
                "role": "assistant",
                "content": [
                    {
                        "type": "text",
                        "text": "我已经记住所有规则、动作函数和案例了，请开始您的指令吧",
                    }
                ],
            }
        )

    def init_oline_asr(self, language):
        self.language = language
        return self.oline_asr_model

    def multimodalinfer(self, prompt, image_path=None):
        """version: 2.0
        通用多模态接口，适用于通义千问平台的多模态模型
        """
        if image_path:
            image_data = self.encode_image(image_path)
            conversation_entry = {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{image_data}"},
                    },
                    {"type": "text", "text": "机器人反馈:执行seewhat()完成"},
                ],
            }
        else:
            conversation_entry = {
                "role": "user",
                "content": [{"type": "text", "text": prompt}],
            }

        self.Multimodalmessages.append(conversation_entry)

        completion = self.multimodal_client.chat.completions.create(
            model=self.multimodel, messages=self.Multimodalmessages
        )

        self.Multimodalmessages.append(
            {
                "role": "assistant",
                "content": [
                    {"type": "text", "text": completion.choices[0].message.content}
                ],
            }
        )

        return completion.choices[0].message.content

    def TaskDecision(self, input: str) -> list:  # 任务决策规划
        """
        决策层模型接口
        input: 用户输入
        """
        if self.international_mode:  # 国际版，调用本地dify应用API
            try:
                chat_response = self.decision_client.create_chat_message(
                    inputs={},
                    query=input,
                    user=self.user,
                    response_mode="blocking",
                )
                chat_response.raise_for_status()
                result = chat_response.json()
                if result.get("answer") is not None:
                    output = [True, result.get("answer"), result.get("conversation_id")]
                else:
                    output = [
                        False,
                        "The model service is abnormal. Check the large model account or configuration options",
                        None,
                    ]
            except Exception as e:
                output = [
                    False,
                    "The model service is abnormal. Check the large model account or configuration options",
                    None,
                ]

        else:  # 国内版,调用百炼大模型平台应用API
            try:

                response = Application.call(
                    api_key=self.tongyi_api_key, app_id=self.tongyi_app_id, prompt=input
                )
                if response.output.text is not None:
                    output = [True, response.output.text, None]
                else:
                    output = [
                        False,
                        "The model service is abnormal. Check the large model account or configuration options",
                        None,
                    ]
            except Exception as e:
                output = [
                    False,
                    "The model service is abnormal. Check the large model account or configuration options",
                    None,
                ]

        return output

    def TaskExecution(
        self,
        input: str,
        map_mapping: str,
        language: str,
        image_path=None,
        video_path=None,
        conversation_id=None,
    ) -> list:  # 执行层模型接口
        """
        执行层模型接口,适用于dify
        input: 用户输入
        map_mapping: 地图映射
        language: 回复语言
        image_path: 图片路径
        vide_path: 视频路径
        conversation_id: 会话id

        return:list
        """
        if image_path is not None:

            with open(image_path, "rb") as file:  # 上传图片
                files = {"file": ("robot-perspective-picture", file, "image/png")}
                response = self.execution_client.file_upload("yahboom", files)
                file_id = response.json().get("id")

            image = [
                {
                    "type": "image",
                    "transfer_method": "local_file",
                    "upload_file_id": file_id,
                }
            ]
            try:
                chat_response = self.execution_client.create_chat_message(
                    inputs={"map_mapping": map_mapping, "language": language},
                    query=input,
                    user=self.user,
                    response_mode="blocking",
                    conversation_id=conversation_id,
                    files=image,
                )
                chat_response.raise_for_status()
                result = chat_response.json()
                if result.get("answer") is not None:
                    output = [True, result.get("answer"), result.get("conversation_id")]
                else:
                    output = [
                        False,
                        "The model service is abnormal. Check the large model account or configuration options",
                        None,
                    ]
            except Exception as e:
                output = [
                    False,
                    "The model service is abnormal. Check the large model account or configuration options",
                    None,
                ]

        elif video_path is not None:
            print('video understanding...')
            with open(video_path, "rb") as file:  # 上传图片
                files = {"file": ("robot-perspective-picture", file, "video/mp4")}
                response = self.execution_client.file_upload("yahboom", files)
                file_id = response.json().get("id")

            video = [
                {
                    "type": "video",
                    "transfer_method": "local_file",
                    "upload_file_id": file_id,
                }
            ]
            try:
                chat_response = self.execution_client.create_chat_message(
                    inputs={"map_mapping": map_mapping, "language": language},
                    query=input,
                    user=self.user,
                    response_mode="blocking",
                    conversation_id=conversation_id,
                    files=video,
                )
                chat_response.raise_for_status()
                result = chat_response.json()
                if result.get("answer") is not None:
                    output = [True, result.get("answer"), result.get("conversation_id")]
                else:
                    print('can not video understanding...')
                    output = [
                        False,
                        "The model service is abnormal. Check the large model account or configuration options",
                        None,
                    ]
            except Exception as e:
                print('failed video understanding...')
                output = [
                    False,
                    "The model service is abnormal. Check the large model account or configuration options",
                    None,
                ]

        
        else:
            try:
                chat_response = self.execution_client.create_chat_message(
                    inputs={"map_mapping": map_mapping, "language": language},
                    query=input,
                    user=self.user,
                    response_mode="blocking",
                    conversation_id=conversation_id,
                )
                chat_response.raise_for_status()

                result = chat_response.json()
                if result.get("answer") is not None:
                    output = [True, result.get("answer"), result.get("conversation_id")]
                else:
                    output = [
                        False,
                        "The model service is abnormal. Check the large model account or configuration options",
                        None,
                    ]
            except Exception as e:
                output = [
                    False,
                    "The model service is abnormal. Check the large model account or configuration options",
                    None,
                ]

        return output

    def oline_asr(self, input_file):
        """
        语音识别接口,兼容通义千问平台paraformer、gummy系列模型
        """
        if self.oline_asr_model in [
            "paraformer-realtime-v2",
            "paraformer-realtime-v1",
            "paraformer-realtime-8k-v2",
            "paraformer-realtime-8k-v1",
        ]:
            output = self.paraformer_asr_inferce(input_file)
            return output
        elif self.oline_asr_model in ["gummy-realtime-v1", "gummy-chat-v1"]:
            output = self.gummy_asr_inferce(input_file)
            return output

    def paraformer_asr_inferce(self, input_file):
        """
        通义千问平台paraformer模型接口
        """
        recognition = Recognition(
            model=self.oline_asr_model,
            format="wav",
            sample_rate=self.oline_asr_sample_rate,
            callback=None,
        )
        result = recognition.call(input_file)
        if result.status_code == HTTPStatus.OK:
            sentences = result.get_sentence()
            if sentences and isinstance(sentences, list):
                return ["ok", sentences[0].get("text", "")]
            else:
                return [
                    "error",
                    "ASR Error: The large model returns an empty result. Please check the account balance or parameter configuration",
                ]
        else:
            return ["error", "ASR Error:" + result.message]

    def gummy_asr_inferce(self, input_file):
        """
        通义千问平台gummy模型接口
        """
        translator = TranslationRecognizerRealtime(
            model=self.oline_asr_model,
            format="wav",
            sample_rate=self.oline_asr_sample_rate,
            translation_target_languages=[self.language],
            translation_enabled=True,
            callback=None,
        )

        result = translator.call(input_file)
        if not result.error_message:
            output = ""
            for transcription_result in result.transcription_result_list:
                output += transcription_result.text
            return ["ok", output]
        else:
            return ["error", result.error_message]

    def init_local_asr_model(self):
        self.model_senceVoice = AutoModel(
            model=self.local_asr_model, trust_remote_code=False, disable_update=True
        )

    def tts_model_init(self, model_type="oline", language="zh"):
        self.current_language = language
        if language == "zh":
            if model_type == "oline":
                self.model_type = "oline"
                if self.tts_supplier == "baidu":
                    self.token = self.fetch_token()
            elif model_type == "local":   
                self.model_type = "local"
                tts_model = self.zh_tts_model
                tts_json = self.zh_tts_json
                self.synthesizer = piper.PiperVoice.load(
                    tts_model, config_path=tts_json, use_cuda=False
                )
        elif language == "en":
            if model_type == "oline":
                self.model_type = "oline"
            elif model_type == "local": 
                self.model_type = "local"
                tts_model = self.en_tts_model
                tts_json = self.en_tts_json
                self.synthesizer = piper.PiperVoice.load(
                    tts_model, config_path=tts_json, use_cuda=False
                )



    def SenseVoiceSmall_ASR(self, input_file, language="zn"):
        res = self.model_senceVoice.generate(
            input=input_file,
            cache={},
            language=language,  # "zn", "en", "yue", "ja", "ko", "nospeech"
            use_itn=False,
        )
        prompt = res[0]["text"].split(">")[-1]
        return ["ok", prompt]

    def voice_synthesis(self, text, path):
        """
        语音合成
        text:合成的文本
        path:保存路径
        返回1:失败 返回0:成功
        """
        if self.model_type == "oline":
            if self.current_language == "zh":
                if self.tts_supplier == "baidu":
                    """
                    百度智能云平台语音合成模型接口
                    """
                    # print('baiduhecheng')
                    TTS_URL = "http://tsn.baidu.com/text2audio"
                    tex = quote_plus(text)
                    params = {
                        "tok": self.token,
                        "tex": tex,
                        "per": self.PER,
                        "spd": self.SPD,
                        "pit": self.PIT,
                        "vol": self.VOL,
                        "aue": 3,
                        "cuid": self.CUID,
                        "lan": "zh",
                        "ctp": 1,
                    }  # lan ctp 固定参数

                    data = urlencode(params)
                    req = Request(TTS_URL, data.encode("utf-8"))
                    # has_error = False
                    try:
                        f = urlopen(req)
                        result_str = f.read()

                        # headers = dict((name.lower(), value) for name, value in f.headers.items())

                    except URLError as err:
                        print("asr http response http code : " + str(err.code))
                        result_str = err.read()
                        # has_error = True
                        return 1
                    with open(path, "wb") as of:
                        of.write(result_str)
                        return 0

                elif self.tts_supplier == "aliyun":
                    """
                    阿里通义语音合成接口
                    """
                    self.synthesizer = SpeechSynthesizer(
                        model=self.oline_tts_model, voice=self.voice_tone, volume=100
                    )
                    audio = self.synthesizer.call(text)
                    if audio is None:
                        return 1
                    else:
                        with open(path, "wb") as f:
                            f.write(audio)
                        return 0
            elif self.current_language == "en":
                Xinghou_speaktts(text)
        elif self.model_type == "local":
            with wave.open(path, "wb") as wav_file:
                wav_file.setnchannels(1)  # 单声道
                wav_file.setsampwidth(2)  # 16位采样
                wav_file.setframerate(self.synthesizer.config.sample_rate)  # 设置采样率
                # 进行文本转语音
                self.synthesizer.synthesize(text, wav_file)


    def openrouter_model_infer(self, prompt, image_path=None):
        """
        使用anythingllm连接openrouter平台大模型:已弃用
        Connect the large model of the openrouter platform using anythingllm
        """
        if image_path:
            image_data = self.encode_image(image_path)
            data = {
                "message": self.system_text["text1"],
                "mode": "chat",
                "attachments": [
                    {
                        "name": "image.png",
                        "mime": "image/png",
                        "contentString": f"data:image/png;base64,{image_data}",
                    }
                ],
                "reset": False,
            }
        else:
            data = {"message": prompt, "mode": "chat"}
        # --- 发送 POST 请求 ---
        response = requests.post(self.chat_endpoint, headers=self.headers, json=data)
        response.raise_for_status()  # 如果请求失败 (状态码 >= 400)，则抛出异常
        # --- 处理响应 ---
        result = response.json()

        return result["textResponse"]

    def fetch_token(self):
        """
        专用于百度语音合成的token生成方法,百度平台有专有的token生成工具
        """
        TOKEN_URL = "http://aip.baidubce.com/oauth/2.0/token"
        SCOPE = "audio_tts_post"  # 有此scope表示有tts能力，没有请在网页里勾选
        params = {
            "grant_type": "client_credentials",
            "client_id": self.baidu_API_KEY,
            "client_secret": self.baidu_SECRET_KEY,
        }
        post_data = urlencode(params)
        post_data = post_data.encode("utf-8")
        req = Request(TOKEN_URL, post_data)
        try:
            f = urlopen(req, timeout=5)
            result_str = f.read()
        except URLError as err:
            print("token http response http code : " + str(err.code))
            result_str = err.read()

        result_str = result_str.decode()
        result = json.loads(result_str)
        if "access_token" in result.keys() and "scope" in result.keys():
            return result["access_token"]

    @staticmethod
    def encode_image(image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    @staticmethod
    def get_ip(network_interface):
        addresses = netifaces.ifaddresses(network_interface)
        if netifaces.AF_INET in addresses:
            for info in addresses[netifaces.AF_INET]:
                if "addr" in info:
                    return info["addr"]


# 录完音，可以直接调用去识别 After recording the audio, it can be directly called for recognition
def rec_wav_music_en():
    global xufei, wsParam
    xufei = ""
    # time1 = datetime.now()
    wsParam = Ws_Param(
        APPID="f12672f1",
        APISecret="NmUyYTRmNTM2MjE3OWJkMDczYzlhZDgz",
        APIKey="8c7b9858dc5e11e8490ce0d09879ad1e",
        AudioFile=record_speech_file,
    )
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()
    ws = websocket.WebSocketApp(
        wsUrl, on_message=on_message, on_error=on_error, on_close=on_close
    )
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    return xufei


def Xinghou_speaktts(context):
    global wsParam
    # 测试时候在此处正确填写相关信息即可运行 Fill in the relevant information correctly here during testing to run
    wsParam = Ws_Param_1(
        APPID="f12672f1",
        APISecret="NmUyYTRmNTM2MjE3OWJkMDczYzlhZDgz",
        APIKey="8c7b9858dc5e11e8490ce0d09879ad1e",
        Text=context,
    )
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url_1()
    ws = websocket.WebSocketApp(
        wsUrl, on_message=on_message_1, on_error=on_error_1, on_close=on_close_1
    )
    ws.on_open = on_open_1
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
