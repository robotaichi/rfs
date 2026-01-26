import https from 'https';
import fs from 'fs';
import path from 'path';

const API_KEY = process.env.GEMINI_API_KEY;
const MODEL = 'gemini-3-pro-image-preview';
const API_URL = `https://generativelanguage.googleapis.com/v1beta/models/${MODEL}:generateContent`;

// 引数からプロンプトと出力パスを取得
const prompt = process.argv[2];
const outputPath = process.argv[3];

if (!prompt || !outputPath) {
    console.error('使用方法: node generate-image.js "<プロンプト>" "<出力パス>"');
    process.exit(1);
}

async function generateImage(prompt) {
    const requestData = {
        contents: [{ parts: [{ text: prompt }] }],
        generationConfig: {
            temperature: 1.0,
            topP: 0.95,
            topK: 40,
            maxOutputTokens: 8192
        }
    };

    return new Promise((resolve, reject) => {
        const requestBody = JSON.stringify(requestData);
        const options = {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-goog-api-key': API_KEY,
                'Content-Length': Buffer.byteLength(requestBody)
            }
        };

        const req = https.request(API_URL, options, (res) => {
            let data = '';
            res.on('data', (chunk) => { data += chunk; });
            res.on('end', () => resolve(JSON.parse(data)));
        });

        req.on('error', reject);
        req.write(requestBody);
        req.end();
    });
}

async function main() {
    console.log(`生成中: ${prompt}`);

    const response = await generateImage(prompt);

    if (response.candidates?.[0]?.content?.parts) {
        for (const part of response.candidates[0].content.parts) {
            if (part.inlineData?.data) {
                const buffer = Buffer.from(part.inlineData.data, 'base64');
                fs.mkdirSync(path.dirname(outputPath), { recursive: true });
                fs.writeFileSync(outputPath, buffer);
                console.log(`✓ 画像を保存しました: ${outputPath}`);
            }
        }
    }
}

main();
