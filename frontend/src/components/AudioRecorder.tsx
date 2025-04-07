import { useState, useRef, useEffect } from 'react';
import { MicrophoneIcon, StopIcon } from '@heroicons/react/24/solid';
import { Button } from '@/components/ui/button';

interface AudioRecorderProps {
  onAudioRecorded: (blob: Blob) => void;
}

export function AudioRecorder({ onAudioRecorded }: AudioRecorderProps) {
  const [isRecording, setIsRecording] = useState(false);
  const [audioURL, setAudioURL] = useState<string | null>(null);
  const [recordingDuration, setRecordingDuration] = useState(0);
  const [transcription, setTranscription] = useState<string | null>(null);
  const [agentResult, setAgentResult] = useState<string | null>(null);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const chunksRef = useRef<Blob[]>([]);
  const timerRef = useRef<number | null>(null);

  useEffect(() => {
    return () => {
      if (timerRef.current) clearInterval(timerRef.current);
      if (mediaRecorderRef.current && mediaRecorderRef.current.state === 'recording') {
        mediaRecorderRef.current.stop();
      }
    };
  }, []);

  async function startRecording() {
    try {
      chunksRef.current = [];
      setRecordingDuration(0);
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });

      const mediaRecorder = new MediaRecorder(stream);
      mediaRecorderRef.current = mediaRecorder;

      mediaRecorder.ondataavailable = (e) => {
        if (e.data.size > 0) {
          chunksRef.current.push(e.data);
        }
      };

      mediaRecorder.onstop = () => {
        if (timerRef.current) clearInterval(timerRef.current);

        const audioBlob = new Blob(chunksRef.current, { type: 'audio/wav' });
        const audioUrl = URL.createObjectURL(audioBlob);
        setAudioURL(audioUrl);
        onAudioRecorded(audioBlob);

        // Stop all tracks
        stream.getTracks().forEach((track) => track.stop());
      };

      mediaRecorder.start();
      setIsRecording(true);

      timerRef.current = window.setInterval(() => {
        setRecordingDuration((prev) => prev + 1);
      }, 1000);
    } catch (err) {
      console.error('Error accessing microphone:', err);
      alert('Error accessing microphone. Please ensure you have granted permission.');
    }
  }

  function stopRecording() {
    if (mediaRecorderRef.current && isRecording) {
      mediaRecorderRef.current.stop();
      setIsRecording(false);
    }
  }

  return (
    <div className="flex flex-col items-center gap-4">
      <div className="flex items-center gap-4">
        {isRecording ? (
          <Button
            className="h-20 w-20 bg-red-500 text-white rounded-lg flex items-center justify-center shadow-lg hover:shadow-xl hover:cursor-pointer transition-all duration-300 active:scale-95"
            onClick={stopRecording}
          >
            <StopIcon className="h-8 w-8" />
          </Button>
        ) : (
          <Button
            className="h-20 w-20 rounded-lg flex items-center justify-center shadow-lg hover:shadow-xl hover:cursor-pointer transition-all duration-300 active:scale-95"
            onClick={startRecording}
          >
            <MicrophoneIcon className="h-8 w-8" />
          </Button>
        )}

        {audioURL && (
          <audio
            src={audioURL}
            controls
            className="w-64 rounded-lg shadow-md hover:shadow-lg transition-all duration-300"
          />
        )}
      </div>

      {transcription && (
        <div className="w-full text-center mt-4">
          <h3 className="text-lg font-semibold">Transcription</h3>
          <p className="text-gray-700">{transcription}</p>
        </div>
      )}

      {agentResult && (
        <div className="w-full text-center mt-4">
          <h3 className="text-lg font-semibold">Agent Result</h3>
          <p className="text-gray-700">{agentResult}</p>
        </div>
      )}
    </div>
  );
}